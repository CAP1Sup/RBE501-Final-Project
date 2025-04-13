classdef SpecialDHRobot < handle

    properties
        name % The name of the robot
        robot % The rigid body tree object
        config % The configuration of the robot
        ee_name % The name of the end effector
    end

    methods

        function obj = SpecialDHRobot(name, dhparams)
            obj.name = name;
            obj.robot = rigidBodyTree;
            n = size(dhparams, 1);
            bodies = cell(n, 1);
            joints = cell(n, 1);

            % Future DH parameters to implement
            % Note that the 3rd column (future link(s)) of the original table was removed
            %self.use_inertia = [p[7] for p in params]
            %self.use_Ia = [p[8] for p in params]
            %self.use_friction = [p[9] for p in params]
            %self.spring_dl = [p[10] for p in params]

            for i = 1:n
                % Link numbers are assumed to be in numerical order
                bodies{i} = rigidBody(['body' num2str(i)]);
                joints{i} = rigidBodyJoint(['jnt' num2str(i)], "revolute");
                setFixedTransform(joints{i}, dhparams(i, 3:6), "mdh");
                bodies{i}.Joint = joints{i};

                if i == 1 % Add first body to base
                    addBody(obj.robot, bodies{i}, "base")
                else % Add current body to previous body by name
                    addBody(obj.robot, bodies{i}, bodies{dhparams(i, 2)}.Name)
                end

            end

            obj.ee_name = bodies{n}.Name;
            obj.config = randomConfiguration(obj.robot);

            showdetails(obj.robot);
        end

        function plot(obj)
            % Plot the robot in a given configuration
            figure(Name = obj.name)
            show(obj.robot);
        end

        function T = fkine(obj, q)
            obj.set_joints(q);
            T = getTransform(obj.robot, obj.config, obj.ee_name, 'base');
        end

    end

    methods (Abstract)
        set_joints(obj, q)
    end

end
