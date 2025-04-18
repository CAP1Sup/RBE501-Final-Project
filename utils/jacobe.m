function J_b = jacobe(S,M,q)
    %JACOBE Calculates the body jacobian for the end effector.
    %   INPUTS:
    %       S: Screw axis matrix expressed in the space frame, where each
    %       column is a screw axis (6xn matrix).
    %       M: Home Transformation matrix expressed in the space frame.
    %       q: Vector of joint variables.
    %   OUTPUT:
    %       J_b: End Effector jacobian expressed in the body frame.

    % Calculating the Space Jacobian:
    J_s = jacob0(S, q);

    % Calculating the CURRENT T from the body to the space frame:
    T_sb = fkine(S, M, q, 'space');
    T_bs = T_inverse(T_sb);

    % Converting the Space Jacobian to a Body Jacobian:
    J_b = adjoint(T_bs) * J_s;
    
end

