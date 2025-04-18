function R = axisangle2rot(omega, theta)
    %axisangle2rot Calcualtes rotation matrix from exponential coordinates.
    %   INPUTS: 
    %       Omega: Unit vector rotation axis.
    %       Theta: Amount of rotation (radians)
    %   OUTPUTS:
    %       R = Rotation matrix
    %   Uses Rodriguez Formula (Equation 3.51 from Modern Robotics textbook)

    omega_skew = skew(omega);
    R = eye(3) + sin(theta)*omega_skew + (1-cos(theta))*omega_skew^2;    
end

