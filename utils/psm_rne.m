function [tau,V,Vdot] = psm_rne(params)
%% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
%
% Inputs: params - a structure containing the following fields:
%           params.g - 3-dimensional column vector describing the acceleration of gravity
%           params.S - 6xn matrix of screw axes (each column is an axis)
%           params.M - 4x4xn home configuration matrix for each link
%           params.G - 6x6xn spatial inertia matrix for each link
%           params.jointPos - n-dimensional column vector of joint coordinates
%           params.jointVel - n-dimensional column vector of joint velocities
%           params.jointAcc - n-dimensional column vector of joint accelerations
%           params.Ftip - 6-dimensional column vector representing the
%           wrench applied at the tip
%
% Output: tau  - n-dimensional column vector of generalized joint forces
%         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
%         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
%

%% Finding Home Matrices (Mi, Mi^-1, Mi,i-1):

% Initializing Variables:
n = length(params.jointPos); % Number of Links
M_0s = zeros(size(params.M)); % Store Home matrices with respect to space frame
M_invs = zeros(size(params.M)); % Stores Inverse Home Matrices

% Finding Home Matrices
M_0s(:, :, 1) = params.M(:, :, 1); % First home matrice M01 is already defined w.r.t. the space frame
for i = 2 : n + 1
    M_previous = M_0s(:, :, i - 1);
    M_next = params.M(:, :, i); % Home matrice w.r.t previous link frame
    Mi = M_previous * M_next;
    M_0s(:, :, i) = Mi;
end

% Finding Inverse Home Matrices:
for i = 1 : n+1
    M_current = M_0s(:, :, i);
    M_current_inv = T_inverse(M_current);
    M_invs(:, :, i) = M_current_inv;
end

%% Calculating Screw Axes in Local Link Frames (Ai):

A = zeros(size(params.S)); % Matrix to store Ai for each joint

for i = 1:n
    % Calculating Ai: 
    Ai = adjoint(M_invs(:, :, i)) * params.S(:, i);
    A(:, i) = Ai;
end

%% Finding Transformation Matrices:

% Forward Transformations (T01, T12, etc):
T_forwards = zeros(size(params.M)); % Stroing Forward Transformations T01, T12...

for i = 1 : n
    T_forward_i = fkine(A(:, i), params.M(:, :, i), params.jointPos(i), 'body');
    T_forwards(:, :, i) = T_forward_i;
end
T_forwards(:, :, end) = params.M(:, :, end); % Last transformation has no joint


% Inverse Transformations for Velocity and Acceleration Formulas in RNE:
T_VA = zeros(4, 4, n);
for i = 1 : n
    T_VA_i = T_inverse(T_forwards(:,:, i));
    T_VA(:, :, i) = T_VA_i;
end


% Inverse Transformations for Wrench Formulas in RNE:
T_wrench = zeros(4, 4, n);

for i = 1 : n
    T_wrench_i = T_inverse(T_forwards(:, :, i+1));
    T_wrench(:, :, i) = T_wrench_i;
end

%% Forward iterations

% Initializing Variables:
V = zeros(6, n+1); % Including Space Frame Velocity
Vdot = zeros(6, n+1); % same

% Caculating Link Velocities:
V(:, 1) = zeros(6, 1); % Velocity of Space Frame
for i = 2 : n+1
    j = i - 1; % Accounting for index shift due to space frame
    V_j = A(:, j) * params.jointVel(j) + adjoint(T_VA(:, :, j)) * V(:, i-1 );
    V(:, i) = V_j;
end


% Calculating Link Accelerations: 
Vdot(4:6, 1) = params.g * -1; % Acceleration of space frame (includes gravity)

for i = 2 : n+1
    j = i-1; % Accounting for index shift due to space frame
    local_acc = A(:, j) * params.jointAcc(j);
    prev_link_acc = adjoint(T_VA(:, :, j))*Vdot(:, i-1);
    centripetal_acc = ad(V(:,i))*A(:,j)*params.jointVel(j); 
    Vdot_j = local_acc + prev_link_acc + centripetal_acc;
    Vdot(:, i) = Vdot_j;
end

%% Backward iterations

% Initializing Variables:
W = zeros(6, n+1); % Links + End Effector
tau = zeros(7, 1);

% Preparing Tip Wrench:
W(:, 4) = params.Ftip; % Assuming it is in the end effector frame


gearbox = [];

for i = n : -1 : 1 % Reverse Direction
    j = i + 1; % index shift for V and V_dot to account for space frame
    
    Gi = params.G(:, :, i);
    Vi = V(:, j);
    Vdoti = Vdot(:, j);
    Ai = A(:, i);
    
    % Wrench Calculation:
    Wi = Gi*Vdoti -...
        ad(Vi)'*Gi*Vi +...
        adjoint(T_wrench(:, :, i))'*W(:, i+1);
    W(:, i) = Wi;
    % Wrench Supported by Joint:
    tau(i) = Wi' * Ai;
end

    %%  Adding elastic and friction torques PSM
    % Note: values for parameters from "Modelling and identification of the da
    % Vinci Research Kit robotic arms"
    % Elastic torques
    % Defining elastic gain matrix for power cables and torsional spring
    K_e = diag([0.129, 0.35, 0, 0.003, 0, 0, 0]);
    tau_e = K_e * params.jointPos;
    % Friction torques
    % Creating viscous friction matrix
    Fv = diag([0.133, 0.136, 2.695, 0.001, 0.028, 0.02, 0.02]);
    Fv(5,6) = 0.005;
    Fv(6,5) = 0.013;
    % Creating static friction matrix
    Fs = diag([0.064, 0.15, 0.496, 0.004, 0.012, 0.004, 0.004]);
    % Calculating torques per joint due to friction
    tau_f = friction(params.jointVel, Fv, Fs, zeros(7,1));
    % Calculating total joint torques
    tau = tau + tau_e + tau_f;

end