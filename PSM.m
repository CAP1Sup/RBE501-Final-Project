classdef PSM < URDFRobot

    methods

        function obj = PSM()
            % PSM Creates the kinematic structure of the Patient Side Manipulator (PSM).
            %
            % Inputs: None
            %
            % Output: robot - the robot structure, created using Matlab's Robotics Toolbox

            % Call the parent constructor
            obj = obj@URDFRobot("PSM", "./models/psm/psm_launch.urdf.xacro", "psm_tool_gripper2_link");
        end

        function obj = set_joints(obj, q)
            % SET_JOINTS Sets the joint coordinates of the robot.
            %
            %   Inputs: q - joint coordinates (1x7 vector)
            %
            %   Output: obj - the robot structure with updated joint coordinates

            % Apply joint angles to the model
            % Frames from: http://dx.doi.org/10.12700/APH.16.8.2019.8.3
            qm = zeros(1, 14);
            qm(1) = 0; % Always 0, angle between base and world
            qm(2) = q(1); % Frame 1
            qm(3) = q(2); % Frame 3
            qm(4) = -q(2); % Frame 6
            qm(5) = q(2); % Frame 8
            qm(6) = q(3); % Frame 9
            qm(7) = q(4); % Frame 12
            qm(8) = q(5); % Frame 13
            qm(9) = 0; % Frame 14, tool yaw
            qm(10) = q(6); % Frame 15, gripper finger 1
            qm(11) = q(7); % Gripper finger 2
            qm(12) = -q(2); % Frame 7
            qm(13) = q(2); % Frame 4
            qm(14) = 0; % Frame 5 or Frame 11?

            % Populate the config with the joint coordinates
            A = num2cell(qm);
            [obj.config.JointPosition] = A{:};
        end

    end

end
