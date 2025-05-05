clear; clc; close all;

addpath("models\")
addpath("utils\")
addpath("traj\")
%% Loading MTM URDF:
robot =  MTM();
robot_bodies = robot.robot.Bodies;
n_bodies = robot.robot.NumBodies;

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
w1 = [0, 0, 1]';
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
       0, 0, 1, h;
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
M_end = M01 * M12 * M23 * M34 * M45 * M56 * M67 * M78;


%% Getting Spatial Inertias for the Links:

chain1_Gs = zeros(6, 6, length(chain1_indeces));

% Chain 1:
for i = 1 : length(chain1_indeces)
    link_i = chain1{i};
    Gi = urdf_to_spatial_inertia(link_i);
    chain1_Gs(:, :, i) = Gi;
end

% Tuned Spatial Inertia Matrices:
G2I = 2; G2m = 0.9;
G2 = diag([G2I, G2I, G2I, G2m, G2m, G2m]);
chain1_Gs(:, :, 2) = G2;

G3I = 2.5; G3m = 0.1;
G3 = diag([G3I, G3I, G3I, G3m, G3m, G3m]);
chain1_Gs(:, :, 3) = G3;

G6I = 0.1; G6m = 0.2;
G6 = diag([G6I, G6I, G6I, G6m, G6m, G6m]);
chain1_Gs(:, :, 6) = G6;

G7I = 0.004; G7m = 0.1;
G7 = diag([G7I, G7I, G7I, G7m, G7m, G7m]);
chain1_Gs(:, :, 7) = G7/2;


%% Load Trajectory:
[q, qd, qdd, meas_tau, est_tau] = load_traj('./traj/mtm/train', 'est_tau_with_cable.csv');


%% RNE - Chain 1

% Initializing Variables to store Results:
rne_simple = zeros(size(est_tau));
rne_f= zeros(size(est_tau));
rne_s = zeros(size(est_tau));
rne_m = zeros(size(est_tau));
rne_fsm = zeros(size(est_tau));

for i = 1 : length(q)
    
    % Loading Into params:
    params_b.g =[0 0 -9.81]; % Gravity Vector [m/s^2]
    params_b.S = Sb;       
    params_b.M = M;
    params_b.jointPos = q(i, :); 
    params_b.jointVel = qd(i, :);
    params_b.jointAcc = qdd(i, :);
    params_b.Ftip = zeros(6,1);
    params_b.G = chain1_Gs;

    % RNE Calculations:
    tau_simple_i = mtm_rne(params_b, 1);
    tau_f_i = mtm_rne(params_b, 2);
    tau_s_i = mtm_rne(params_b, 3);
    tau_m_i = mtm_rne(params_b, 4);
    tau_fsm_i = mtm_rne(params_b, 5);
    

    % Store Results:
    rne_simple(i, :) = tau_simple_i';
    rne_f(i, :) = tau_f_i';
    rne_s(i, :) = tau_s_i';
    rne_m(i, :) = tau_m_i';
    rne_fsm(i, :) = tau_fsm_i';
end

%% Plotting: 

% Plotting RNE VS Paper Results:
figure('Name','Joint‑level torque comparison', 'WindowState','maximized')
tiledlayout(4, 2,"TileSpacing","compact");   % 8 tiles → last one remains blank

for j = [1 : 7]
    nexttile;

    plot(rne_fsm(1:2000, j),       'LineWidth', 3);       % RNE solid
    hold on;
    plot(est_tau(1:2000, j),'--',  'LineWidth', 3);       % estimate dashed
    hold on;
    plot(meas_tau(1:2000, j),'-.',  'LineWidth', 3);       % estimate dashed
    hold off;

    title(sprintf('Joint %d',j));
    xlabel('Iterations');           
    ylabel('Torque [N·m]');
    legend({'RNE','Estimated', 'Measured'},'Location','southeast');
    grid on;
end
exportFigurePDF('MTM_Torques')
sgtitle('MTM - RNE Vs. Estimated (Wang et al.) Vs. Measured Torques');
hold off;


% Plotting With and without Friction and Elastic Torques:
figure('Name','Friction and Elastic Torque', 'WindowState','maximized');
tiledlayout(4,2,"TileSpacing","compact");   % 8 tiles → last one remains blank

gear_ratio_list = [63.41, 49.88, 59.73, 10.53, 33.16, 33.16, 16.58]';

for j = 1:7
    nexttile;

    plot(rne_simple(1:500, j),'--',  'LineWidth',3);     
    hold on;
    plot(rne_f(1:500, j),'--',  'LineWidth',3);  
    hold on;
    plot(rne_s(1:500, j),'-.',  'LineWidth',3);
    hold on;
    plot(rne_m(1:500, j),'-.',  'LineWidth',3);
    hold on;
    plot(rne_fsm(1:500, j),       'LineWidth',3);    
    hold off;

    
    title(sprintf('Joint %d',j));
    xlabel('i');           
    ylabel('Torque [N·m]');
    legend({'None Added', 'Friction', 'Spring', 'Motor Inertia', 'Friction + Spring + Motor Inertia'},'Location','southeast');
    grid on;
end

%sgtitle('RNE Torques - Impact of Friction, Springs, and Motor Inertia');
hold off;

exportFigurePDF('Friction and Elastic Torque')


%% Error Calculations:
rmse_est = rmse(rne_fsm, est_tau)
rmse_meas = rmse(rne_fsm, meas_tau)


rmse_list = zeros(1, 7);
average_list = zeros(1, 7);
rmse_normalized_list = zeros(1, 7);
for j = 1:7
    rmse_j = rmse(meas_tau(:,j), rne_fsm(:, j));
    rmse_est_j = rmse(est_tau(:, j), rne_fsm(:, j));
    average_j = max(abs(meas_tau(:, j)));
    average_est_j= max(abs(est_tau(:, j)));

    rmse_list(j) = rmse_j;
    rmse_est_list(j) = rmse_est_j;
    rmse_normalized_list(j) = rmse_j / average_j;
    rmse_normalized_est_list(j) = rmse_est_j / average_est_j;
end
rmse_normalized_percentage = rmse_normalized_list * 100;
rmse_normalized_est_percentage = rmse_normalized_est_list * 100;

fprintf('\nRMSE Percentage vs Measured Torques:\n')
disp(rmse_normalized_percentage);
fprintf('\nRMSE Percentage vs Estimated Torques:\n')
disp(rmse_normalized_est_percentage);



