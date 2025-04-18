function J = jacob0(S, q)
    %JACOB0 Calculates space jacobian.
    %   INPUTS:
    %       S = Matrix of screw axis columns. 
    %       q = vector of current joint variables (radians)
    %   OUTPUTS:
    %       J = Jacobian (6 x n), where n is the number of joints

    % Initializing Variables:
    T_prod = eye(4);
    num_joints = size(S, 2);
    J = zeros(6, num_joints);
    
    for i = 1 : num_joints
        % Adjoint of the product of preceeding Transformation Matrices:
        T_prod_adj = adjoint(T_prod);

        % Finding Jacobian for current joint index:
        J_i = T_prod_adj * S(:, i);
        J(:, i) = J_i;

        % Updating product of Transformation Matrices for the next joint:
        T_i = twist2ht(S(:,i), q(i));
        T_prod = T_prod * T_i;
    end


end

