function T_inv = T_inverse(T)
    %T_INVERSE Calculates the inverse of a transformation matrix.
    %   INPUT:
    %       T: Homogenous transformation matrix (4x4 Matrix)
    %   OUTPUT:
    %       T_inverted: Homogenous transformation matrix (4x4 Matrix)

    % Extracting From T:
    R = T(1:3, 1:3);
    p = T(1:3, 4);

    % Inverting R and p:
    R_inv = R';
    p_inv = - R_inv * p;

    % Composing Inverse Transformation Matrix:
    T_inv = zeros(4);
    T_inv(1:3, 1:3) = R_inv;
    T_inv(1:3, 4) = p_inv;
    T_inv(4, 4) = 1;
end

