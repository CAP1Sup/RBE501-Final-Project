function G = urdf_to_spatial_inertia(link)
    %URDF_TO_SPATIAL_INERTIA Converts urdf inertia and mass infomation into
    %a spatial inertia matrix.
    %   INPUT:
    %       link: Body object derived from URDF
    %   OUTPUT:
    %       G: Spatial Inertia (6x6 matrix)

    % Extracting mass and Inertia fields from link:
    m = link.Mass;
    I_vec = link.Inertia; % [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]

    % Converting to sub matrices:
    I = [I_vec(1), I_vec(4), I_vec(5);
         I_vec(4), I_vec(2), I_vec(6);
         I_vec(5), I_vec(6), I_vec(3)];
    mI = eye(3) * m;
    
    % Combining to get Spatial Inertia:
    G = zeros(6);
    G(1:3, 1:3) = I;
    G(4:6, 4:6) = mI;
    
end