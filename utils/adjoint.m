function AdT = adjoint(T)
    %ADJOINT Calculates adjoint transformation matrix of T
    %   INPUT:
    %       T = Transformation matrix in SE(3)
    %   OUTPUT:
    %       AdT = Adjoint Transformation Matrix (6x6)
    %   Using Definition 3.20 of the Modern Robotics (2017) textbook.
    
    % Extracting from T:
    R = T(1:3, 1:3);
    p = T(1:3, 4);

    % Forming Matrix:
    p_skew = skew(p);
    AdT = [R, zeros(3,3); p_skew*R, R];   

end

