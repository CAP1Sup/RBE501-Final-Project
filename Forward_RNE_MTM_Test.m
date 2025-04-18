% Testing MTM Forward Kinematics
clear; clc;
addpath('utils');
%% Defining Chain 1 (q1, q2, q3, q4, q5, q6, q7)

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


% Home Matrices (USING DH FRAMES FOR NOW, need to change to COM frames)
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
       0, 0, 1, 0;
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


% Spatial Inertial Matrices (ARBITRARY VALUES FOR NOW):
G1 = zeros(6,6); 
m1 = 1;
width = 1;
L = 1;
Ixx1 = m1*(width^2+L^2)/12;
Iyy1 = m1*(width^2+L^2)/12;
Izz1 = m1*(L^2+width^2)/12;
G1(1:3,1:3) = diag([Ixx1 Iyy1 Izz1]);
G1(4:6,4:6) = m1 * eye(3);
G2 = G1; 
G3 = G2; 
G4 = G2; 
G5 = G2; 
G6 = G2; 
G7 = G2;
Gb = cat(3, G1, G2, G3, G4, G5, G6, G7);


%% Performing RNE on Chain 1: (Ignore Torque Results for now)

% Assigning Joint Parameters:
q1 = 1; q1_dot = 1; q1_ddot = 0;
q2 = 2; q2_dot = 2; q2_ddot = 0;
q3 = 3; q3_dot = 3; q3_ddot = 0;
q4 = 4; q4_dot = 4; q4_ddot = 0;
q5 = 5; q5_dot = 5; q5_ddot = 0;
q6 = 6; q6_dot = 6; q6_ddot = 0;
q7 = 7; q7_dot = 7; q7_ddot = 0;

qb = [q1, q2, q3, q4, q5, q6, q7]; %qb = "q basis"
qb_dot = [q1_dot, q2_dot, q3_dot, q4_dot, q5_dot, q6_dot, q7_dot];
qbddot = [q1_ddot, q2_ddot, q3_ddot, q4_ddot, q5_ddot, q6_ddot, q7_ddot];


% Loading Into params:
params.g =[0 0 -9.81]; % Gravity Vector [m/s^2]
params.S = Sb;       
params.M = M;
params.G = Gb;
params.jointPos = qb; % Current Joint Variables (arbitrarily set to zero)
params.jointVel = qb_dot; % Current Joint Velocities (arbitrarily set to zero)
params.jointAcc = qbddot; % Current Joint Accelerations (arbitrarily set to zero)
params.Ftip = zeros(6,1);     % Wrench applied at the tip


% RNE Calculation:
[tau_b,V_b,Vdot_b] = rne(params);


% Print Results:
fprintf('RESULTS FOR BASIS LINKS:\nVelocities:\n');
disp(V_b);
fprintf('Accelerations:\n');
disp(Vdot_b);

%% Finding Joint Parameters for Parallel Links:

% Relationships from paper:
q3p = q2 + q3; %q3'
q3pp = -q3; % q3''
q3ppp = q3; % q3'''
qa = [q1, q3p, q3pp, q3ppp, q4, q5, q6, q7]; % qa = "q additional". Note also including necessary basis joints (q1, q4, q5, q6, q7)

% Joint Velocities:
q3p_dot = q2_dot + q3_dot;
q3pp_dot = -q3_dot;
q3ppp_dot = q3_dot;
qa_dot = [q1_dot, q3p_dot, q3pp_dot, q3ppp_dot, q4_dot, q5_dot, q6_dot, q7_dot];

% Joint Accelerations: 
q3p_ddot = q2_ddot + q3_ddot;
q3pp_ddot = -q3_ddot;
q3ppp_ddot = q3_ddot;
qa_ddot = [q1_ddot, q3p_ddot, q3pp_ddot, q3ppp_ddot, q4_ddot, q5_ddot, q6_ddot, q7_ddot];


%% Defining Chain 2 (q1, q3', q3'', q3''', q4, q5, q6, q7)
% New Screw Axes:
w3p = w2;
w3pp = w2;
w3ppp = w2;

wa = [w1, w3p, w3pp, w3ppp, w4, w5, w6, w7]; % Combining With necesssary basis axes (q1, q4, q5, q6, q7)

p3p = p2;
p3pp = [-Lbf, 0, -Lbp]';
p3ppp = [-Lbf, 0, (-Lbp -La)]';
pa = [p1, p3p, p3pp, p3ppp, p4, p5, p6, p7];

Sa = revolute_screw(wa, pa);


% Home Matrices (USING DH FRAMES FOR NOW, need to change to COM frames)
M01 = [1, 0, 0, 0;
       0, 1, 0, 0;
       0, 0, 1, -Lbp;
       0, 0, 0, 1];

M13p = [-1, 0, 0, 0; %M13'
         0, 0, 1, 0;
         0, 1, 0, 0;
         0, 0, 0, 1];

M3p3pp = [0, 1, 0, Lbf; %M3'3''
         -1, 0, 0, 0;
          0, 0, 1, 0;
          0, 0, 0 1];

M3pp3ppp = [1, 0, 0, La; %M3''3'''
            0, 1, 0, 0;
            0, 0, 1, 0;
            0, 0 0, 1];

M3ppp4 = [0, 0, -1, -h; %M3'''4
          1, 0, 0, Lf-Lbf;
          0, -1, 0, 0;
          0, 0, 0, 1];


M_a = cat(3, M01, M13p, M3p3pp, M3pp3ppp, M3ppp4, M45, M56, M67, M78);


% Spatial Inertial Matrices (ARBITRARY VALUES FOR NOW):
G3p = zeros(6,6); 
m1 = 1;
width = 1;
L = 1;
Ixx1 = m1*(width^2+L^2)/12;
Iyy1 = m1*(width^2+L^2)/12;
Izz1 = m1*(L^2+width^2)/12;
G3p(1:3,1:3) = diag([Ixx1 Iyy1 Izz1]);
G3p(4:6,4:6) = m1 * eye(3);
G3pp = G3p; 
G3ppp = G3p; 

Ga = cat(3, G1, G3p, G3pp, G3ppp, G4, G5, G6, G7);


%% Performing RNE on Chain 1: (Ignore Torque Results for now)

% Loading Into params:
params.g =[0 0 -9.81]; % Gravity Vector [m/s^2]
params.S = Sa;       
params.M = M_a;
params.G = Ga;
params.jointPos = qa; % Current Joint Variables (arbitrarily set to zero)
params.jointVel = qa_dot; % Current Joint Velocities (arbitrarily set to zero)
params.jointAcc = qa_ddot; % Current Joint Accelerations (arbitrarily set to zero)
params.Ftip = zeros(6,1);     % Wrench applied at the tip


% RNE Calculation:
[tau_a,V_a,Vdot_a] = rne(params);


% Print Results:
fprintf('\n\nRESULTS FOR Parallel Chain:\nVelocities:\n');
disp(V_a);
fprintf('Accelerations:\n');
disp(Vdot_a);


%% Testing if Velocities and Accelerations Match:

% Extracting only results for the shared links between the two chains 1, 4, 5, 6, 7, 8:
b_common_ind = [1, 3, 4, 5, 6, 7, 8];
V_b_common = V_b(:, b_common_ind);
Vdot_b_common = Vdot_b(:, b_common_ind);

a_common_ind = [1, 4, 5, 6, 7, 8, 9];
V_a_common = V_a(:, a_common_ind);
Vdot_a_common = Vdot_a(:, a_common_ind);

% Finding Difference between Results:
V_difference = V_b_common - V_a_common;
Vdot_difference = Vdot_b_common - Vdot_a_common;

% Printing Results:
fprintf('\n\nThe difference in velocities between shared links is:\n')
disp(V_difference);
fprintf('\nThe difference in accelerations between shared links is:\n')
disp(Vdot_difference);





