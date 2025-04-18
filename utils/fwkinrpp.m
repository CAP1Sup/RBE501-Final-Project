function T = fwkinrpp(q)
%FKINSCARA Calculates forward kinematics for a RPP robot
%   Input: q is a 3x1 vector with the joint parameters
%   Output: Overall Transformation Matrix for the robot 

% Defining Link lengths:
L1 = 0.5; % [m]

% Extracting joint parameters:
theta_1 = q(1); d_2 = q(2); d_3 = q(3);

% DH Parameters (calculated by hand):
dh_thetas = [theta_1, pi/2, -pi/2];
dh_ds = [L1, d_2, d_3];
dh_as = [0, 0, 0];
dh_alphas = [0 pi/2, 0];

% Calculating intermediate transformation_matrices:
a1 = tdh(dh_thetas(1), dh_ds(1), dh_as(1), dh_alphas(1));
a2 = tdh(dh_thetas(2), dh_ds(2), dh_as(2), dh_alphas(2));
a3 = tdh(dh_thetas(3), dh_ds(3), dh_as(3), dh_alphas(3));


% Calculating Overall transformation matrix
T = a1 * a2 * a3;
end

