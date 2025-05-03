clear; clc;

addpath("models\")
addpath("utils\")
%% Loading MTM URDF:
robot =  MTM();
matlab_robot = rigidBodyTree("DataFormat","column");
matlab_robot.BaseName = 'world';
robot_bodies = robot.robot.Bodies;
n_bodies = robot.robot.NumBodies;

%% Creating Copy

for i = 1: length(robot_bodies)
    bodyCopy = copy(robot_bodies{i});
    % if isempty(robot_bodies{i}.Parent)
    %     addBody(matlab_robot, bodyCopy, "base");
    % else
        addBody(matlab_robot, bodyCopy, robot_bodies{i}.Parent.Name);
    % end
end

matlab_robot.Gravity = robot.robot.Gravity;


%% Mapping  URDF Information to Frames Assigned per DVRK Paper We are Replicating:

% Identifying Chain 1 Indeces:
L1_ind = 3; % "mtm_outer_yaw_link"
L2_ind = 4; % "mtm_back_parallel_link"
L3_ind = 5; % "mtm_bottom_parallel_link"
L4_ind = 6; % "mtm_wrist_platform_link"
L5_ind = 7; % "mtm_writst_pitch_link"
L6_ind = 8; % "mtm_wrist_yaw_link"
L7_ind = 9; % "mtm_wrist_roll_link"
chain1_indeces = [L1_ind, L2_ind, L3_ind, L4_ind, L5_ind, L6_ind, L7_ind];
chain1 = robot_bodies(chain1_indeces);


%% Defining Chain 1 (Basis Chain - q1, q2, q3, q4, q5, q6, q7):

% Link Lengths:
Lbp = 0.2154;
Lbf = 0.1;
La = 0.2794;
Lf = 0.3645;
h = 0.1056;


% Screw Axes:
w1 = [0, 1, 0]';
w2 = [0, 1, 0]';
w3 = [0, 1, 0]';
w4 = [0, 0, 1]';
w5 = [0, 1, 0]';
w6 = [0, 0, 1]';
w7 = [1, 0, 0]';
wb = [w1, w2, w3, w4, w5, w6, w7];

p1 = [0, 0, 0]';
p2 = [0, 0, -Lbp]';
p3 = [0, 0, -(Lbp + La)]';
p4 = [-Lf, 0, 0]';
p5 = [-Lf, 0, -(Lbp + La - h)]';
p6 = p4;
p7 = [0, 0, -(Lbp + La - h)]';
pb = [p1, p2, p3, p4, p5, p6, p7];

Sb = revolute_screw(wb, pb);


% Home Matrices (Using frames from Yan Wang 2019 Paper)
M01 = [1, 0, 0, 0;
       0, 1, 0, 0;
       0, 0, 1, -Lbp;
       0, 0, 0, 1];

M12 = [0, -1, 0, 0;
       0, 0, 1, 0;
       -1, 0, 0, 0;
       0, 0, 0, 1];

M23 = [0, -1 0, La;
       1, 0, 0, 0;
       0, 0, 1, 0;
       0, 0, 0 1];

M34 = [1, 0, 0, Lf;
       0, 0, 1, h; % Before I was missing the h
       0, -1, 0, 0;
       0, 0 0, 1];

M45 = [1, 0, 0, 0;
       0, 0, -1, 0;
       0, 1, 0, 0;
       0, 0, 0, 1];

M56 = [0, -1, 0, 0;
       0, 0, 1, 0;
       -1, 0, 0, 0;
       0, 0, 0, 1];

M67 = [-1, 0, 0, 0;
       0, 0, 1, 0;
       0, 1, 0, 0;
       0, 0, 0, 1];

M78 = [1, 0, 0, 0;
       0, 1, 0, 0;
       0, 0, 1, 0;
       0, 0, 0, 1]; % Adding an extra frame for the end-effector

M = cat(3, M01, M12, M23, M34, M45, M56, M67, M78);

%% Getting Spatial Inertias for the Links:

chain1_Gs = zeros(6, 6, length(chain1_indeces));

% Chain 1:
for i = 1 : length(chain1_indeces)
    link_i = chain1{i};
    Gi = urdf_to_spatial_inertia(link_i);
    chain1_Gs(:, :, i) = Gi;
end


%% Defining Joint Parameters - Chain 1:

 % Assigning Joint Parameters Randomly:
    min = -1; max = 1;
    q1 = q_rand(1, min, max); q1_dot = q_rand(1, min, max); q1_ddot = q_rand(1, min, max);
    q2 = q_rand(1, min, max); q2_dot = q_rand(1, min, max); q2_ddot = q_rand(1, min, max);
    q3 = q_rand(1, min, max); q3_dot = q_rand(1, min, max); q3_ddot = q_rand(1, min, max);
    q4 = q_rand(1, min, max); q4_dot = q_rand(1, min, max); q4_ddot = q_rand(1, min, max);
    q5 = q_rand(1, min, max); q5_dot = q_rand(1, min, max); q5_ddot = q_rand(1, min, max);
    q6 = q_rand(1, min, max); q6_dot = q_rand(1, min, max); q6_ddot = q_rand(1, min, max);
    q7 = q_rand(1, min, max); q7_dot = q_rand(1, min, max); q7_ddot = q_rand(1, min, max);


    % Defining Joint Parameters for Chain 1:
    qb = [q1, q2, q3, q4, q5, q6, q7]; %qb = "q basis"
    qb_dot = [q1_dot, q2_dot, q3_dot, q4_dot, q5_dot, q6_dot, q7_dot];
    qb_ddot = [q1_ddot, q2_ddot, q3_ddot, q4_ddot, q5_ddot, q6_ddot, q7_ddot];


%% RNE - Chain 1

% Loading Into params:
params_b.g =[0 0 -9.81]; % Gravity Vector [m/s^2]
params_b.S = Sb;       
params_b.M = M;
params_b.jointPos = qb; % Current Joint Variables (arbitrarily set to zero)
params_b.jointVel = qb_dot; % Current Joint Velocities (arbitrarily set to zero)
params_b.jointAcc = qb_ddot; % Current Joint Accelerations (arbitrarily set to zero)
params_b.Ftip = zeros(6,1);
params_b.G = chain1_Gs;

% RNE Calculation:
[tau, V_b,Vdot_b] = rne(params_b);

fprintf('The torques calculated with RNE: \n')
disp(tau)


%% MATLAB version

% Relationships from paper:
q3p = q2 + q3; %q3'
q3pp = -q3; % q3''

% Joint Velocities:
q3p_dot = q2_dot + q3_dot;
q3pp_dot = -q3_dot;


% Joint Accelerations: 
q3p_ddot = q2_ddot + q3_ddot;
q3pp_ddot = -q3_ddot;

% Combining Joints:
q_c = [qb, q3p, q3pp];
q_c_dot = [qb_dot, q3p_dot, q3pp_dot];
q_c_ddot = [qb_ddot, q3p_ddot, q3pp_ddot];



matlab_torques = matlab_robot.inverseDynamics(q_c', q_c_dot', q_c_ddot');

fprintf('The Matlab results for the same torqes is:\n')
disp(matlab_torques(1:7))