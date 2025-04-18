function V_b = twistspace2body(V_s,T)
    %TWISTSPACE2BODY Converts a space twist to a body twist.
    %   INPUTS:
    %       V_s: Twist defined in the space frame (6-vector)
    %       T: Homogenous Transformation from {s} to {b} (4x4 matrix)
    %   OUTPUTS:
    %       V_b: Twist defined in the body frame (6-vector)

    % Calculating Adjoint Transfomation Matrix of T_inverse:
    T_inv = T_inverse(T);
    ad_T_inv = adjoint(T_inv);

    % Converting Twist frame using adjoint matrix:
    V_b = ad_T_inv * V_s;
end

