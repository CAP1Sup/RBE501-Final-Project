function T = twist2ht(S,theta)
    %TWIST2HT Calculates a transormation matrix given a screw axis and
    %theta.
    %   INPUTS:
    %       s = screw axis
    %       theta = rotation amount (radians)
    %   OUTPUT:
    %       T = Homogenous Transformation Matrix
    %   Uses Equation 3.88 from the Modern Robotics (2017) textbook.

    % Extracting angular and linear portions of screw axis:
    omega = [S(1); S(2); S(3)]; % Ensures it is a column vector
    v = [S(4); S(5); S(6)];

    % Checking if screw axis is valid:
    omega_n = norm(omega); v_n = norm(v);
    if ~(omega_n == 1 || ((omega_n == 0) & (v_n == 1)))
        error('INVALID SCREW AXIS')
    end

    % Calculating T:
    omega_skew = skew(omega);
    rot = axisangle2rot(omega, theta);
    translation = (eye(3) * theta + (1 - cos(theta)) * omega_skew + ...
        (theta - sin(theta))* omega_skew^2) * v;

    T = [rot, translation];
    T = [T; 0, 0, 0, 1];
    
end

