% RBE 501: Robot Dynamics
% Authors: Group
% Date: 4/18/25
% Description: This script compares our calculated inverse dynamics to
    % Peter Corke's Robotics Toolbox
clear, clc, close all
addpath('utils');
plotOn = true;

%% Create the manipulator (DH)
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

n_joints = 8;
robot = SerialLink([RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', pi/2), ...
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2), ...
                    RevoluteMDH('a', -l_2L3, 'd', -l_2H1, 'alpha', pi/2, 'offset', 0), ...
                    RevoluteMDH('a', 0, 'd', -0, 'alpha', pi/2, 'offset', 0),...
                    RevoluteMDH('a', l_2L2, 'd', -0, 'alpha', 0, 'offset', 0),...
                    PrismaticMDH('a', l_2L3,'alpha', -pi/2, 'qlim', [0, 0.416 + 0.25419], 'theta', 0, 'offset', -l_2H3), ...
                    RevoluteMDH('a', 0, 'd', l_tool, 'alpha', 0, 'offset', 0),...
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2)], ...
                    'name', 'PSM');

% Display the manipulator in the home configuration
q = zeros(1, n_joints);
q(1, 3) = 0; % intermediate link, should always be 0
q(1, 2) = 0;
q(1, 4) = -q(1, 2);
q(1, 5) = q(1, 2);
qlim = [-1.605 -0.93556  0 -0.93556 -0.93556 -0.002444 -3.0456 -3.0414; %lower
        1.5994 0.94249 0 0.94249 0.94249 0.24001 3.0485 3.0528]'; %upper
robot.teach(q);

%% Calculate the Foward Kinematics
% Part A - Calculate the screw axes
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints

S = [0, 1, 0, 0, 0, 0;
    1, 0, 0, 0, 0, 0; % 100% I need to fix the dh on the robot
    0, 0, 1, 0, 0, 0;
    -1, 0, 0, -cross([-1, 0, 0], [0, -l_2L3, l_2H1]);
    -1, 0, 0, -cross([-1, 0, 0], [0, l_2L2-l_2L3, l_2H1]);
    0, 0, 0, 0, 0, -1;
    0, 0, -1, -d3, 0, 0;
    -1, 0, 0, 0, -d5, d3]';

% Part B - Calculate the forward kinematics with the Product of Exponentials formula
% First, let us calculate the homogeneous transformation matrix M for the
% home configuration

M = [1, 0, 0, 0;
     0, 0, -1, d3;
     0, 1, 0, d6;
     0, 0, 0, 1];


%% Calculate the Inverse Dynamics (RNE)
% Populate params (input to inverse dynamics - RNE)
params.g = [0, 0, -9.81];
params.S = S;
params.jointPos = q;
params.jointVel = zeros(6);
params.jointAcc = zeros(6);
params.Ftip = zeros(6);

% Calculate the transformations between successive frames (M's)
% Calculate T's to help
T01 = twist2ht(S(:,1), 0);
T02 = T01 * twist2ht(S(:,2),0);
T02_prime = T02 * twist2ht(S(:,3), 0);
T02_prime_prime = T02_prime * twist2ht(S(:,4), 0);
T03 = T02_prime_prime * twist2ht(S(:,5), 0);
T04 = T03 * twist2ht(S(:,6), 0);
T05 = T04 * twist2ht(S(:,7), 0);
T06 = T05 * twist2ht(S(:,8), 0);
T07 = M;

% Calculate M's using T's
M01 = T01;
M12 = pinv(M01) * T02;
M2_2_prime = pinv(T02) * T02_prime;
M2_prime_2_prime_prime = pinv(T02_prime) * T02_prime_prime;
M2_prime_prime_3 = pinv(T02_prime_prime) * T03;
M34 = pinv(T03) * T04;
M45 = pinv(T04) * T05;
M56 = pinv(T05) * T06;
M67 = pinv(T06) * T07;
params.M = cat(3, M01, M12, M2_2_prime, M2_prime_2_prime_prime, M2_prime_prime_3, M34, M45, M56, M67);


% TODO: Calculate the proper spatial inertia matrices 
    % Find rotational inertia matrices (not just the zero matrix)
    % Ensure principal axes of inertia tensor match the link's frame
m1 = 1.4705; 
m2 = 0.98494; 
m2_prime = 0.17841; 
m2_prime_prime = 2.091; 
m3 = 0.22491; 
m4 = 0.0; 
m5 = 0.00025784; 
m6 = 0.0003225;
G1 = [zeros(3,3), zeros(3,3);
      zeros(3,3), m1 * eye(3)];
G2 = [zeros(3,3), zeros(3,3);
      zeros(3,3), m2 * eye(3)];
G2_prime = [zeros(3,3), zeros(3,3);
      zeros(3,3), m2_prime * eye(3)];
G2_prime_prime = [zeros(3,3), zeros(3,3);
      zeros(3,3), m2_prime_prime * eye(3)];
G3 = [zeros(3,3), zeros(3,3);
      zeros(3,3), m3 * eye(3)];
G4 = [zeros(3,3), zeros(3,3);
      zeros(3,3), m4 * eye(3)];
G5 = [zeros(3,3), zeros(3,3);
      zeros(3,3), m5 * eye(3)];
G6 = [zeros(3,3), zeros(3,3);
      zeros(3,3), m6 * eye(3)];
Glist = cat(3, G1, G2, G2_prime, G2_prime_prime, G3, G4, G5, G6);
params.G = Glist;

% Call RNE function to calculate tau, V, and Vdot
[tau,V,Vdot] = rne(params);

disp("V:");
disp(V);
disp("Vdot");
disp(Vdot);
disp("Calculated tau:");
disp(tau);

tau_gravity = robot.gravload(q);
disp("Correct tau")
disp(tau_gravity);

