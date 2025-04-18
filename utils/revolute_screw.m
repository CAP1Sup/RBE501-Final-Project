function S = revolute_screw(rot_axes,p)
    %SCREW Calculates screw axis for a revolute joint.
    %   INPUTS:
    %       rot_axes = Matrix of axes of rotation for each joint
    %       p = Matrix of translations for each screw axis
    %   OUTPUTS: 
    %       S = Matrix of screw axes
    
    % Initializing Screw Matrix:
    num_joints = size(rot_axes, 2);
    S = zeros(6, num_joints);

    for joint = 1 : num_joints
        % Extracting rotation axis and location for the joint
        w_i = rot_axes(:, joint); 
        p_i = p(:, joint);
        
        % Calculating linear velocity portion of screw axis:
        w_i_skew = skew(w_i);
        v_i = -w_i_skew*p_i; % Note: this assumes p_dot is 0

        % Composing the screw axis for the joint:
        s_i = [w_i; v_i];
        S(:, joint) = s_i;
    end
end

