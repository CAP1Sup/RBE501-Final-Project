function T = fkine(S, M, q, frame)
    %FKINE Calculates forward kinematics using product of exponentials.
    %   INPUTS:
    %       S = Matrix of screw axis columns. (1st --> nth Joint)
    %       M = Home Transformation Matrix
    %       q = Vector of joint Variables (radians)
    %       frame = 'body' or 'space' frame that screw axes are defined in.
    %   OUTPUTS:
    %       T = Overall Transormation Matrix
    %   Uses equation 4.14 of Modern Robotics (2017) textbook.
    
    T_prod = eye(4); % Initiliazing

    % Finding transformation matrix for each axis:
   for i = 1: size(S, 2)
       T_i = twist2ht(S(:, i), q(i));
       T_prod = T_prod*T_i;
   end

   % Calculating Transformation Matrix Depending on frame specified:
   switch upper(frame)
       case 'SPACE'
           T = T_prod * M;
       case 'BODY'
           T = M * T_prod;
   end

end

