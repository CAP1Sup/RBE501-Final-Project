function T = fwkinscara(q)
%FKINSCARA Calculates forward kinematics for a scara robot
%   Input: q is a 4x1 vector with the joint parameters
%   Output: Overall Transformation Matrix for the robot 

% Defining Link lengths:
L1 = 0.5; % [m]
L2 = 0.5; % [m]
L4 = 0.1; % [m]

% Extracting joint parameters:
theta_1 = q(1); theta_2 = q(2); d_3 = q(3); theta_4 = q(4);

% DH Parameters (calculated by hand):
dh_thetas = [theta_1, theta_2, 0, -theta_4];
dh_ds = [0, 0, d_3, L4];
dh_as = [L1, L2, 0, 0];
dh_alphas = [0, pi, 0, 0];

% Calculating intermediate transformation_matrices:
a1 = tdh(dh_thetas(1), dh_ds(1), dh_as(1), dh_alphas(1));
a2 = tdh(dh_thetas(2), dh_ds(2), dh_as(2), dh_alphas(2));
a3 = tdh(dh_thetas(3), dh_ds(3), dh_as(3), dh_alphas(3));
a4 = tdh(dh_thetas(4), dh_ds(4), dh_as(4), dh_alphas(4));

% Calculating Overall transformation matrix
T = a1 * a2 * a3 * a4;

% Alternative Method:
% 
% 
% % Creating a matrix with all of the intermediate transformation matrices
% % stored
% t_matrices = zeros(4, 4, length(q));
% for t_num = 1:length(q)
%     t_matrices(1:4, 1:4, t_num) = tdh(dh_thetas(t_num), dh_ds(t_num), ...
%         dh_as(t_num), dh_alphas(t_num));
% end
% 
% % Calculating the overal transformation matrix
% T = t_matrices(1:4,1:4,1) * t_matrices(1:4,1:4,2) *...
%     t_matrices(1:4,1:4,3) * t_matrices(1:4,1:4,4);
end

