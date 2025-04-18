function J_a = jacoba(S,M,q)
    %JACOBA Calculates the analytic jacobian. Maps joint velocities to the
    %end effector velocity expressed in the space frame. 
    %   INPUTS:
    %       S: Screw axis matrix expressed in the space frame, where each
    %       column is a screw axis (6xn matrix).
    %       M: Home Transformation matrix expressed in the space frame.
    %       q: Vector of joint variables.
    %   OUTPUT:
    %       J_a: Analytic Jacobian

    % Calculating Space Jacobian:
    J_s = jacob0(S, q);

    % Calculating the CURRENT T from the space frame to the end-effector:
    T_sb = fkine(S, M, q, 'space');
    p = T_sb(1:3, 4); % Extracting translation
    p_skew = skew(p);

    % Extracting velocity and angular portions of space jacobian:
    J_s_w = J_s(1:3, :);
    J_s_v = J_s(4:6, :);

    % Converting Space Jacobain to Analytic Jacobian:
    J_a = J_s_v - (p_skew * J_s_w);
    
end

