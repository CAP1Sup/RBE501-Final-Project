% RBE 501: Robot Dynamics
% Authors: Callahan Henry (and group)
% Date: 4/18/25
% Description: This script compares the forward kinematics using the
% provided DH method and our PoE calculations

% RBE 501 - Robot Dynamics - Spring 2025
% Homework 2, Problem 1
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/06/2025
clear, clc, close all
addpath('../utils');

plotOn = true;
nTests = 30; % number of random test configurations

%% Create the manipulator
% Link length values (meters)
d3 = 0.516;
d5 = 0.0156;
d6 = d5 - 0.00091;

l_RCC = 0.4318; % meters
l_tool = 0.4162;
l_2H1 = 0.14454;
l_2L3 = 0.04009;
l_2L2 = 0.516;
l_2H2 = 38.08;
l_2H3 = l_RCC - l_2H1;

n_joints = 10;
% robot = SerialLink([RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', pi/2), ...
%                     RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2), ...
%                     RevoluteMDH('a', -l_2L3, 'd', -l_2H1, 'alpha', pi/2, 'offset', 0), ...
%                     RevoluteMDH('a', 0, 'd', -0, 'alpha', pi/2, 'offset', 0),...
%                     RevoluteMDH('a', l_2L2, 'd', -0, 'alpha', 0, 'offset', 0),...
%                     PrismaticMDH('a', l_2L3,'alpha', -pi/2, 'qlim', [0, 0.416 + 0.25419], 'theta', 0, 'offset', -l_2H3), ...
%                     RevoluteMDH('a', 0, 'd', l_tool, 'alpha', 0, 'offset', 0),...
%                     RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2), ...
%                     RevoluteMDH('a', 0.00091, 'd', 0, 'alpha', -pi/2, 'offset', pi/2), ...
%                     RevoluteMDH('a', 0, 'd', 0, 'alpha', 0, 'offset', 0)],...
%                     'name', 'PSM');

% Display the manipulator in the home configuration
q = zeros(1, 10);

% q(1, 3) = 0; % intermediate link, should always be 0
% q(1, 2) = 0;
% q(1, 4) = -q(1, 2);
% q(1, 5) = q(1, 2);

qlim = [-1.605 -0.93556 0 -0.93556 -0.93556 -0.002444 -3.0456 -3.0414 -3.0481 -3.0498; %lower
        1.5994 0.94249 0 0.94249 0.94249 0.24001 3.0485 3.0528 3.0376 3.0399]'; %upper
% q(1, 3) = 0.416;
robot.teach(q);

%% Part A - Calculate the screw axes
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints

S = [0, 1, 0, 0, 0, 0; % 1
     1, 0, 0, 0, 0, 0; % 2
     -1, 0, 0, -cross([-1, 0, 0], [0, -l_2L3, l_2H1]); % 2'
     -1, 0, 0, -cross([-1, 0, 0], [0, l_2L2 - l_2L3, l_2H1]); % 2''
     0, 0, 0, 0, 0, -1; % 3
     0, 0, -1, -d3, 0, 0; % 4
     -1, 0, 0, 0, -d5, d3; % 5
     0, -1, 0, d6, 0, 0; % 6
     0, -1, 0, d6, 0, 0]';

% %% Part B - Calculate the forward kinematics with the Product of Exponentials formula
% % First, let us calculate the homogeneous transformation matrix M for the
% % home configuration

M = [1, 0, 0, 0;
     0, 0, -1, d3;
     0, 1, 0, d6;
     0, 0, 0, 1];

q = zeros(1, n_joints);
T = fkine(S, M, q, "SPACE");
robot.teach(q);
assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
fprintf('\nHome config test passed successfully.\n');

fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Test the forward kinematics for 100 random sets of joint variables
for ii = 1:nTests
    fprintf(repmat('\b', 1, nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii / nTests * 100));

    % Generate a random configuration
    q = [qlim(1, 1) + (qlim(1, 2) - qlim(1, 1)) * rand(), ...
             qlim(2, 1) + (qlim(2, 2) - qlim(2, 1)) * rand(), ...
             qlim(3, 1) + (qlim(3, 2) - qlim(3, 1)) * rand(), ...
             qlim(4, 1) + (qlim(4, 2) - qlim(4, 1)) * rand(), ...
             qlim(5, 1) + (qlim(5, 2) - qlim(5, 1)) * rand(), ...
             qlim(6, 1) + (qlim(6, 2) - qlim(6, 1)) * rand(), ...
             qlim(7, 1) + (qlim(7, 2) - qlim(7, 1)) * rand(), ...
             qlim(8, 1) + (qlim(8, 2) - qlim(8, 1)) * rand(), ...
             qlim(9, 1) + (qlim(9, 2) - qlim(9, 1)) * rand(), ...
             qlim(10, 1) + (qlim(10, 2) - qlim(10, 1)) * rand()];

    % q = zeros(1, n_joints);
    q(1, 2) = q(1, 2) - pi / 32;
    q(1, 3) = 0; % intermediate link, should always be 0
    q(1, 4) = q(1, 2);
    q(1, 5) = -q(1, 2);

    % q = zeros(1, n_joints);
    % q(1, 2) = pi/2;

    % Calculate the forward kinematics
    T = fkine(S, M, q, "SPACE");

    if plotOn
        robot.teach(q);
        title('Forward Kinematics Test');
    end

    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');
