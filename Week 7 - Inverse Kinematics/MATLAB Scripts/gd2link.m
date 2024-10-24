%% ENME480 IK Script
% The purpose of this script is to demonstrate the differences between
% motions in joint space and cartesian space. The script will also run a
% numerical approach called Cyclic Coordinate Descent (CCD) to solve the
% inverse kinematics problem.

%% SETUP
clear all; close all;
% simulation variables
% desired end effector position
pDes = [3;2];
% grid resolution for plotting
dx = .01;
% workspace definition
x = -5:dx:5; y = x;
% initial CCD guess
q = [pi;pi;pi/4];
% GD gain for CCD
delta = 5e-3;
% maximum acceptable error in the final end effector position
errFloor = .1;
% max number of steps for ccd
maxSteps = 1e3;

%% necessary variables
% use symbolic variables for derivative claculation later on
syms tht1 tht2 tht3 real
% two link robot; store the state as [θ1, θ2]
nJoints = 2;
s = [tht1; tht2];

%% Helpful functions
% DH transform function
% inputs:
%   alp - angle z(n+1) rotates about x(n+1)
%   r - distance to move the origin along x(n+1)
%   d - distance to move the origin along z(n)
%   tht - angle x(n+1) rotates about z(n)
% outputs:
%   H - transform from frame (n) to (n+1)
M = @(alp,r,d,tht) [...
    cos(tht), -sin(tht)*cos(alp), sin(tht)*sin(alp), r*cos(tht);
    sin(tht), cos(tht)*cos(alp), -cos(tht)*sin(alp), r*sin(tht);
    0, sin(alp), cos(alp), d;
    0, 0, 0, 1];

% DH table function
% inputs:
%   s - vector of θ values (2 by default) to populate the table
% outputs:
%   D - a populated DH matrix for the planar robot
% this function assumes that α, d and r are fixed; you can replace any
% value in this table with an input to actuate the robot in a different
% way. The script *should* be able to track that.
DH = @(s) [... %alp, r, d, tht
    0, 3, 0, s(1);
    0, 2, 0, s(2)];
symDH = DH(s);

%% Distance-to-goal plotting logic
% first, define the workspace of the robot and get the
% distance-to-goal for every point within the workspace
[x,y] = meshgrid(x,y);
d = sqrt((x - pDes(1)).^2 + (y - pDes(2)).^2);
% now plot
subplot(2,2,1); hold on; grid on;
title("Cartesian Error in a Cartesian Perspective")
surf(x,y,d,'EdgeColor','none')
colormap winter; colorbar;
xlabel("x (cm)"); ylabel("y (cm)");
scatter(pDes(1),pDes(2), 120,'k*')

% use matlabs symbolic math toolbox to create two functions which give the
% end effector position and jacobian given a set of joint angles
symM = eye(4);
for i = 1:size(symDH,1)
    symM = symM * M(symDH(i,1),symDH(i,2),symDH(i,3),symDH(i,4));
end
% create a symbolic function to return the final transform as a function of joint angles
doFK = matlabFunction(symM(1:2,4),'vars',{s});
% get workspace cartesian coord of end effector in terms of joint space
x2 = matlabFunction(symM(1,4)); y2 = matlabFunction(symM(2,4));
% derivative of end effector pos w/r.t. its x and y coordinates
xJS = matlabFunction(jacobian(symM(1,4),s),'vars',{s}); 
yJS = matlabFunction(jacobian(symM(2,4),s),'vars',{s});

% create a matrix of end effector distance-to-goal values from the joint
% space perspective
phi = linspace(0, 2*pi, sqrt(numel(x))); 
[phi1, phi2] = meshgrid(phi,phi);
dSym = sqrt((x2(phi1,phi2) - pDes(1)).^2 + (y2(phi1,phi2) - pDes(2)).^2);

% plot joint space end effector error
subplot(2,2,3); hold on; grid on; 
surf(phi1, phi2, dSym,'EdgeColor','none');  % our nice contours are gone!
colormap winter; colorbar;
title("Cartesian Error from Joint Space Perspective")
xlabel("\theta_1"); ylabel("\theta_2")
xlim([0, 2*pi]); ylim([0, 2*pi]);

% initial end effector positions
subplot(2,2,1);
endEffectorPosCart = scatter3(x2(q(1),q(2)), y2(q(1),q(2)), max(d,[],'all'), 120, 'r^', 'filled');
subplot(2,2,3);
endEffectorPosJoint = scatter3(q(1), q(2), max(dSym, [], 'all'), 120, 'r^', 'filled');
subplot(2,2,2); hold on; grid on; 
xlim([min(x,[],'all'),max(x,[],'all')]);
ylim([min(y,[],'all'),max(y,[],'all')])
subplot(2,2,4); hold on; grid on;
xlim([0 maxSteps]); ylim([0 inf]);
title("Error Over Time")
xlabel("Step (t)")
ylabel("Cartesian Error (m)")

%% CCD Solver Loop
% first get the position of each joint as a function of the angles
joint1 = M(symDH(1,1),symDH(1,2),symDH(1,3),symDH(1,4));
joint2 = joint1*M(symDH(2,1),symDH(2,2),symDH(2,3),symDH(2,4));
jointPos = matlabFunction([joint1(1,4), joint1(2,4);joint2(1,4),joint2(2,4)],'vars', s);

% define our objective function as the derivative of distance between the 
% current and desired end effector positions
obj = matlabFunction(jacobian(sqrt((joint2(1,4) - pDes(1))^2 + (joint2(2,4) - pDes(2))^2),s), 'vars', {s});
%iterate through the joints, moving along the derivative to zero at each
%point
% init error and step counter
error = inf; nSteps = 0; errS = [];
while error > errFloor && nSteps < maxSteps
    nSteps = nSteps + 1;
    % main gd loop
    % get gradient at the current state
    de = obj(q);
    % store the old state for plotting
    qOld = q;
    % run GD on the current joint
    q = q - delta*de;
    % get the new end effector position
    p = jointPos(q(1),q(2));
    % calculate the error for the new state
    err = sqrt((p(2,1) - pDes(1)).^2 + (p(2,2) - pDes(2)).^2);
    % store the error for plotting
    errS = [errS, err];

    % stepwise plotting
    % make an animated plot of the robot and where the end effector is
    % joint space error
    subplot(2,2,3);
    set(endEffectorPosJoint,'Visible','off')
    endEffectorPosJoint = scatter3(q(1), q(2), max(dSym, [], 'all'), 120, 'r^', 'filled');
    plot3([qOld(1), q(1)], [qOld(2), q(2)], max(dSym,[],'all')*[1,1], 'r-', 'LineWidth', 1.5)
    % robot position
    subplot(2,2,2); cla; 
    pOld = jointPos(qOld(1),qOld(2));
    pOld = pOld(2,:);
    robotPos = jointPos(q(1),q(2));
    scatter(0,0,48,'ro','filled')
    scatter(pDes(1),pDes(2),120,'bo','filled')
    scatter(robotPos(1,1),robotPos(1,2),48,'ro','filled')
    scatter(robotPos(2,1),robotPos(2,2),48,'ro','filled')
    plot([0; robotPos(:,1)],[0; robotPos(:,2)],'r-','LineWidth',1.5);
    % cartesian error
    subplot(2,2,1);
    set(endEffectorPosCart,'Visible','off')
    endEffectorPosCart = scatter3(x2(q(1),q(2)), y2(q(1),q(2)), max(d,[],'all'), 120, 'r^', 'filled');
    plot3([pOld(1), robotPos(2,1)], [pOld(2), robotPos(2,2)], max(d,[],'all')*[1,1], 'r-', 'LineWidth', 1.5)
    subplot(2,2,4); cla;
    plot(errS, 'LineWidth', 1.5); 
    drawnow

    if err <= errFloor
        fprintf("Solution found in " + num2str(nSteps) + " steps w/error " + num2str(err) + "\n")
        nSteps = maxSteps + 1; % break isnt??
        break
    end
end