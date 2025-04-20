classdef MTM < URDFRobot

    methods

        function obj = MTM()
            % MTM Creates the kinematic structure of the Master Tool Manipulator (MTM).
            %
            % Inputs: None
            %
            % Output: robot - the robot structure, created using Matlab's Robotics Toolbox

            % Call the parent constructor
            obj = obj@URDFRobot("MTM", "./models/mtm/mtm_launch.urdf.xacro", "mtm_ee_link");
        end

        function obj = set_joints(obj, q)
            % SET_JOINTS Sets the joint coordinates of the robot.
            %
            %   Inputs: q - joint coordinates (1x7 vector)
            %
            %   Output: obj - the robot structure with updated joint coordinates

            % Pulley radii
            r3 = 14.01; % mm
            r4 = 20.92; % mm

            % Apply the joint angles to the model
            % Frames from: https://doi.org/10.1109/LRA.2019.2927947
            qm = zeros(1, 10);
            qm(1) = q(1);
            qm(2) = q(2);
            qm(3) = q(3);
            qm(4) = q(2) + q(3);
            qm(5) = 0; %
            qm(6) = -q(3);
            qm(7) = q(4) - r3 / r4 * q(3);
            qm(8) = q(3);
            qm(9) = q(6);
            qm(10) = q(7);

            % Populate the config with the joint coordinates
            A = num2cell(qm);
            [obj.config.JointPosition] = A{:};
        end

    end

end
