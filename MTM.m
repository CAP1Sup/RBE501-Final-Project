classdef MTM < SpecialDHRobot

    methods

        function obj = MTM()
            % MAKE_MTM Creates the kinematic structure of the Master Tool Manipulator (MTM).
            %
            %   Inputs: None
            %
            %   Output: robot - the robot structure, created using Peter Corke's
            %   robotics toolbox

            MM_2_M = 0.001;
            l_b2p = 215.4 * MM_2_M;
            l_arm = 279.4 * MM_2_M;
            l_b2f = 100 * MM_2_M;
            l_fa = 364.5 * MM_2_M;
            %h = 105.6 * MM_2_M;

            % Define link number
            % Shifted for Matlab indexing
            L_b = 1;
            L_1 = 2;
            L_2 = 3;
            L_30 = 4;
            L_31 = 5;
            L_32 = 6;
            L_4 = 7;
            L_5 = 8;
            L_6 = 9;
            L_7 = 10;

            %M_4 = 11;

            % Define spring delta L
            dlN = NaN;

            %q = qmd5;
            %r_s = 0.0075;
            %h_s = 0.1035;
            %l_r = 0.0613;
            %q_o = 23.0/180.0 * pi;
            %l = sqrt(r_s ^ 2 + h_s ^ 2 - 2 * r_s * h_s * cos(pi + q_o - q));
            %d_l = l - l_r;
            %r_f = r_s * h_s * sin(pi + q_o - q) / l;
            dl5 = 0; %r_f * d_l % * 27.86

            % DH
            dh = [[L_b, -1, 0, 0, 0, 0, false, false, false, dlN];
                   [L_1, L_b, 0, 0, -l_b2p, 0, true, false, true, dlN];
                   [L_2, L_1, 0, -pi / 2, 0, pi / 2, true, false, true, dlN];
                   [L_30, L_2, l_arm, 0, 0, pi / 2, true, false, true, dlN];
                   [L_31, L_1, 0, -pi / 2, 0, pi, true, false, true, dlN];
                   [L_32, L_31, l_b2f, 0, 0, - pi / 2, true, false, true, dlN];
                   [L_4, L_30, l_fa, -pi / 2, 0.151, 0, true, false, true, dlN];
                   [L_5, L_4, 0, pi / 2, 0, 0, true, false, true, dl5];
                   [L_6, L_5, 0, -pi / 2, 0, pi / 2, true, false, true, dlN];
                   [L_7, L_6, 0, -pi / 2, 0, pi, true, false, true, dlN]];

            % Assistive frame for incorporating the joint coordinate of motor 4
            % [M_4, L_b, 0, 0, 0, q4, false, true, true, dlN]];

            % Call the parent constructor
            obj = obj@SpecialDHRobot("MTM", dh);
        end

        function obj = set_joints(obj, q)
            % SET_JOINTS Sets the joint coordinates of the robot.
            %
            %   Inputs: q - joint coordinates (1x7 vector)
            %
            %   Output: obj - the robot structure with updated joint coordinates

            % dVRK ROS joint coordinates
            qd2 = q(2);
            qd3 = -q(2) + q(3);
            qd4 = 0.6697 * q(2) - 0.6697 * q(3) + q(4);

            % Calculate the model joint coordinates
            qm = zeros(1, 10);
            qm(1) = q(1);
            qm(2) = qd2;
            qm(3) = qd3;
            qm(4) = qd3 + qd2;
            qm(5) = -qd3;
            qm(6) = q(3);
            qm(7) = qd4;
            qm(8) = q(5);
            qm(9) = q(6);
            qm(10) = q(7);

            % Populate the config with the joint coordinates
            A = num2cell(qm);
            [obj.config.JointPosition] = A{:};
        end

    end

end
