function T = trot(theta,axis)
%trot Calculates a rotation matrix based on a rotation angle and axis
%   Inputs: theta (radians), axis (x, y, z)

% Simplifying
c_theta = cos(theta);
s_theta = sin(theta);

% Formulas from section 3.2 of Modern Robotics Textbook (2017)
if axis == 'x'
    T = [1, 0, 0;
        0, c_theta, -s_theta;
        0, s_theta, c_theta];

elseif axis == 'y'
    T = [c_theta, 0, s_theta;
        0, 1, 0;
        -s_theta, 0, c_theta];

elseif axis == 'z'
    T = [c_theta, -s_theta, 0;
        s_theta, c_theta, 0;
        0, 0, 1];

else
    fprintf('ERROR: incorrect axis inputted');

end

