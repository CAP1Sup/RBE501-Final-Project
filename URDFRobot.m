classdef URDFRobot < handle

    properties
        name % The name of the robot
        robot % The rigid body tree object
        config % The configuration of the robot
        ee_name % The name of the end effector body
        plot_axes % The axes for plotting
    end

    methods

        function obj = URDFRobot(name, urdf_file, ee_name)
            obj.name = name;
            obj.robot = importrobot(urdf_file);

            obj.ee_name = ee_name;
            obj.config = randomConfiguration(obj.robot);
            obj.set_joints(zeros(1, obj.robot.NumBodies));
            obj.plot_axes = obj.robot.show(obj.config, Frames = "off");

            showdetails(obj.robot);
        end

        function plot(obj, q)
            % Save the current camera view
            cam = obj.plot_axes.CameraPosition;
            cam_view = obj.plot_axes.CameraViewAngle;

            % Plot the robot in a given configuration
            obj.set_joints(q);
            obj.plot_axes = obj.robot.show(obj.config, Parent = obj.plot_axes, Frames = "off");

            % Restore the camera view
            obj.plot_axes.CameraPosition = cam;
            obj.plot_axes.CameraViewAngle = cam_view;
        end

        function T = fkine(obj, q)
            obj.set_joints(q);
            T = getTransform(obj.robot, obj.config, obj.ee_name, obj.robot.BaseName);
        end

    end

    methods (Abstract)
        set_joints(obj, q)
    end

end
