function skew_sym_matrix = skew(vector)
    %skew Returns the skey symmetric matrix for a 3x1 vector.
    %   It uses equation 3.30 from the MODERN ROBOTICS MECHANICS, PLANNING,
    %   AND CONTROL (Lynch and Park 2017) textbook:
    %   skew_sym_matrix:    [0,    -x_3,    x_2]
    %                       [x_3,    0,    -x_1]
    %                       [-x_2,    x_1,    0]

    skew_sym_matrix = [0, -vector(3), vector(2);
        vector(3), 0, -vector(1);
        -vector(2), vector(1), 0];
end