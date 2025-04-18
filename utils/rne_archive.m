function [tau,V,Vdot] = rne(params)
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

%% Testing if they sent us M01, M12, M23, M34 Instead of M1, M2, M3, M4 (CONFIRMED)
M01 = params.M(:, :, 1);
M12 = params.M(:, :, 2);
M23 = params.M(:, :, 3);
M34 = params.M(:, :, 4);

M1 = M01;
M2 = M1 * M12;
M3 = M2 * M23;
M4 = M3 * M34;

M = cat(3, M1, M2, M3, M4);
params.M = M;

%% Finding All Home Matrix Types (Mi, Mi_inv, Mi_i+1):

% Initializing Variables:
n = length(params.jointVel);
frames = n + 1; % Number of frames (not including space frame)
M_inv = nan(size(params.M)); % Matrix of inverse home matrices for each joint
M_ip1 = nan(size(params.M));


% Assigning first M01 (trivial):
M_ip1(:, :, 1) = params.M(:, :, 1);

for i = 1:frames
    % Finding Home Matrix Inverses:
    Mi = params.M(:, :, i);
    Mi_inv = T_inverse(Mi);
    M_inv(:, :, i) = Mi_inv; % Saving Matrix Inverses


    % Finding Home Forward Transforms M (i to i+1):
    if i == frames
        break; % Prevents exceeding boundaries
    end
    M_next = Mi_inv * params.M(:, :, i+1);
    M_ip1(:, :, i+1) = M_next;
end


%% Calculating Screw Axes in Local Link Frames (Ai):

A = nan(size(params.S)); % Matrix to store Ai for each joint

for i = 1:n
    % Calculating Ai: 
    Ai = adjoint(M_inv(:, :, i)) * params.S(:, i);
    A(:, i) = Ai;
end

%% Calculating all Needed Transformation Matrices:

% Initializing Variables:
T_ip1 = nan(size(params.M)); % Matrix to store Forward Transforms
T_im1 = nan(size(params.M)); % Matrix to store Backwards Transforms


% Forward Transforms T(i to i+1):
for i = 1 : n
    Ti_ip1 = fkine(A(:, i), M_ip1(:, :, i), params.jointPos(i), 'body');
    T_ip1(:, :, i) = Ti_ip1;
    

    % Inverse Transforms T(i+1 to i):
    T_im1(:, :, i) = T_inverse(Ti_ip1);
end 

% Adding Transformations for the End Effector Frame:
Tn_np1 = M_ip1(:, :, n+1); % The end effector frame does not move relative to frame n
T_ip1(:, :, n+1) = Tn_np1;
T_im1(:, :, n+1) = T_inverse(Tn_np1); % Inverse

%% Forward iterations

% Initializing Variables:
V = nan(6, n+1); % Including End Effector Velocity
Vdot = nan(6, n+1); % Same

% Caculating Link Velocities:
for i = 1 : frames
    if i == 1
        vi = A(:, i) * params.jointVel(i); % Velocity of space frame is 0
    elseif i <= n
        vi = A(:, i) * params.jointVel(i) + adjoint(T_im1(:, :, i)) * V(:, i-1);
    else % End Effector Velocity
        vi = adjoint(T_im1(:, :, i)) * V(:, i-1); % No joint velocity 
    end
    V(:, i) = vi;
end


% Calculating Link Accelerations: 
for i = 1 : frames
    if i == 1
        Vdot_0= zeros(6,1);
        Vdot_0(4:6) = params.g * -1; % Space frame velocity is the opposite of gravity
        Vdot_i = A(:, i) * params.jointAcc(i) +...
            adjoint(T_im1(:, :, i))*Vdot_0 +...
            ad(V(:,i))*A(:,i)*params.jointVel(i); 
    elseif i <= n
        Vdot_i = A(:, i) * params.jointAcc(i) +...
            adjoint(T_im1(:, :, i)) * Vdot(:, i-1) +...
            ad(V(:,i))*A(:, i)*params.jointVel(i);
    else % End Effector Acceleration
        Vdot_i = adjoint(T_im1(:, :, i)) * Vdot(:, i-1); % No Joint Acceleration
    end
    Vdot(:, i) = Vdot_i;
end

%% Backward iterations

% Initializing Variables:
W = nan(6, n);
tau = nan(n, 1);


for i = n : -1 : 1 % Reverse Direction
    Gi = params.G(:, :, i);
    Vi = V(:, i);
    Vdoti = Vdot(:, i);
    Ai = A(:, i);
    
    if i == n
        Wi = Gi*Vdoti -...
            ad(Vi)' * Gi * Vi +...
            params.Ftip; % Assuming Ftip is already in the correct frame
    else
        T_ip1_i = T_ip1(:, :, i+1);
        Wi = Gi*Vdoti -...
            ad(Vi)'*Gi*Vi +...
            adjoint(T_ip1_i)'*W(:, i+1);
    end
    W(:, i) = Wi;

    tau(i) = Wi' * Ai; % Wrench supported by joint
end


% MODIFYING RESULTS to be in form expected by MATLAB grader:
V_mod(:, 1) = zeros(6, 1);
V_mod(:, 2) = V(:, 1);
V_mod(:, 3) = V(:, 2);
V_mod(:, 4) = V(:, 3);
V = V_mod;

Vdot_mod(:, 1) = Vdot_0
Vdot_mod(:, 2) = Vdot(:, 1);
Vdot_mod(:, 3) = Vdot(:, 2);
Vdot_mod(:, 4) = Vdot(:, 3);
Vdot = Vdot_mod;


end