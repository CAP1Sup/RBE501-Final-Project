function T_friction = friction(qdot, Fv, Fc, Fo)
    %FRICTION Calculates friction torques for all joints using the
    %following model:
    %   T = Fv * qdot + Fc *sgn(qdot) + Fo. 
    %   INPUTS:
    %       qdot: Joint Velocities (n x 1 vector)
    %       Fv: Viscous Friction Coefficients (diagonal n x n matrix)
    %       Fc: Coulomb Friction Coefficients (diagonal n x n matrix)
    %       Fo: Friction Offset (n x 1 vector) Note: Accounts for potential
    %       differences in friction depending on direction of rotation
    %   OUTPUTS:
    %       T_friction: Friction Torques for each joint (n x 1 vector)

    viscous_component = Fv * qdot;
    coulomb_component = Fc * sign(qdot);
    T_friction = viscous_component + coulomb_component + Fo;


    % Likely Need to include jacobian to map these to joints later
end

