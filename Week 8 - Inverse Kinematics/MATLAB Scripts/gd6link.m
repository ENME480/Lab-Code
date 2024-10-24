%% ENME480 IK Script
% The purpose of this script is to demonstrate the differences between
% motions in joint space and cartesian space. The script will also run a
% numerical approach called Gradient Descent (GD) to solve the
% inverse kinematics problem.

%% SETUP
clear all; close all;
% simulation variables
% DH table function
% inputs:
%   s - vector of θ values (2 by default) to populate the table
% outputs:
%   D - a populated DH matrix for the planar robot
% this function assumes that α, d and r are fixed; you can replace any
% value in this table with an input to actuate the robot in a different
% way. The script *should* be able to track that.
% revolute robot
DH = @(s) [... %alp, r, d, tht
    0, .8, 0, s(1);
    0, .5, 0, s(2);
    0, .25, 0, s(3);
    0, 1, 0, s(4);
    0, .5, 0, s(5);
    0, 1.2, 0, s(6)];
% prismatic robot
% DH = @(s) [...
%     -pi/2, 0, 0, 0;
%     -pi/2, 0, s(1), pi/2;
%     0, 0, s(2), 0];
% number of actuated joints in the robot
nJoints = 6;
% desired end effector position
pDes = [3;2];
% grid resolution for plotting
dx = .01;
% workspace definition
x = -5:dx:5; y = x;
% initial CCD guess
state = 2*pi*rand(1,nJoints);
% GD gain for CCD
delta = 5e-3;
% maximum acceptable error in the final end effector position
errFloor = .1;
% max number of steps for ccd
maxSteps = 1e3;

%% necessary variables
% use symbolic variables for derivative claculation later on
syms q [1 nJoints] real
% two link robot; store the state as [θ1, θ2]
s = q;

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

symDH = DH(s);

%% Distance-to-goal plotting logic
% first, define the workspace of the robot and get the
% distance-to-goal for every point within the workspace
[x,y] = meshgrid(x,y);
d = sqrt((x - pDes(1)).^2 + (y - pDes(2)).^2);

% use matlabs symbolic math toolbox to create two functions which give the
% end effector position and jacobian given a set of joint angles
symM = eye(4); jointPos = [];
for i = 1:size(symDH,1)
    symM = symM * M(symDH(i,1),symDH(i,2),symDH(i,3),symDH(i,4));
    jointPos = [jointPos; symM(1,4), symM(2,4)];
end
% create a symbolic function to return the final transform as a function of joint angles
doFK = matlabFunction(symM(1:2,4),'vars',{s});

%% CCD Solver Loop
% first get the position of each joint as a function of the angles
jointPos = matlabFunction(jointPos,'vars', {s});

% define our objective function as the derivative of distance between the 
% current and desired end effector positions
obj = matlabFunction(jacobian(sqrt((symM(1,4) - pDes(1))^2 + (symM(2,4) - pDes(2))^2),s), 'vars', {s});
%iterate through the joints, moving along the derivative to zero at each
%point
% init error and step counter
error = inf; nSteps = 0; errS = [];
subplot(1,2,1); hold on; grid on;
xlim([-5 5]); ylim([-5 5]);
subplot(1,2,2); hold on; grid on;
title("Error Over Time")
xlabel("Step (t)")
xlim([0 maxSteps]);
ylim([0 inf])
while error > errFloor && nSteps < maxSteps
    nSteps = nSteps + 1;
    % main ccd loop
    % get gradient at the current state
    de = obj(state);
    % store the old state for plotting
    stateOld = state;
    % run GD on each joint
    state = state - delta*de;
    % get the new end effector position
    p = jointPos(state);
    % calculate and store the error for the new state
    err = sqrt((p(end,1) - pDes(1)).^2 + (p(end,2) - pDes(2)).^2);
    errS = [errS;err];

    % stepwise plotting
    % make an animated plot of the robot and where the end effector is
    % robot position
    subplot(1,2,1); cla; 
    pOld = jointPos(stateOld);
    pOld = pOld(2,:);
    robotPos = jointPos(state);
    robotPos = [0,0;robotPos];
    scatter(0,0,48,'ro','filled')
    scatter(pDes(1),pDes(2),120,'bo','filled')
    for i = 1:size(robotPos,1)-1
        plot([robotPos(i,1), robotPos(i+1,1)],[robotPos(i,2), robotPos(i+1,2)], 'r-', 'LineWidth', 1.5)
        scatter(robotPos(i+1,1), robotPos(i+1,2), 48, 'ro', 'filled')
    end
    subplot(1,2,2); cla
    plot(errS, 'LineWidth', 1.5); 
    drawnow;

    if err <= errFloor
        fprintf("Solution found in " + num2str(nSteps) + " steps w/error " + num2str(err) + "\n")
        nSteps = maxSteps + 1; % break isnt??
        break
    end
end