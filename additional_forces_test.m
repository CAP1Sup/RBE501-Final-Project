%% Purpose: Test Friction, Spring, and other Force Functions
clear; clc;
%% Defining Constants for MTM:

% Noting Which Joint Coordinates to Model Friction for (Per Paper):
qb_f_log = ones(7, 1); % qb = [q1, q2, ... q7]
qa_f_log = [1, 1, 0]'; % qa = [q3', q3'', q3''']
qm_f_log = [0, 0, 0, 1, 0, 0, 0]'; % qm = [qm1, qm2, ...qm7]. 
qc_f_log = [qb_f_log; qa_f_log; qm_f_log];

% Assigning Arbitrary Friction Constants for now:
random_frictions = rand(size(qc_f_log, 1), 3);
random_frictions(~qc_f_log, :) = 0; % Removing Friction Constants from Unmodeled Joints

% Formatting Friction Constants per paper:
Fc = diag(random_frictions(:, 1));
Fv = diag(random_frictions(:, 2));
Fo = random_frictions(:, 3);

% Spring:

% Motor Inertia:

%% Defining Constants for PSM:

% Friction:

% Spring:

% Motor Inertia:

%% Calculating Contribution to Motor Torques

% Test joint Velocities:
qdot = rand(17, 1);
friction_torques = friction(qdot, Fv, Fc, Fo);
fprintf('The calculated friction torques are: \n')
disp(friction_torques);


