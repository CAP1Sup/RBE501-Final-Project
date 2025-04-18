function omega_b = angvelocityspace2body(omega_s,R)
    %ANGLEVELOCITYSPACE2BODY Converts space angular velocity to body
    %angular velocity.
    %   INPUTS:
    %       omega_s: The angular velocity expressed in the space frame (3x1
    %       vector)
    %       R: The rotation matrix from the space to the body frame (3x3
    %       matrix)
    %   OUTPUTS:
    %       omega_b: The angular velocity expressed in the body frame (3x1
    %       vector).
    
    % We are converting a vector from one frame to another, therefore:
    omega_b = R' * omega_s; % R' is the Rotation from {b} to {s}
end

