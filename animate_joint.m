clear;

robot = MTM(); % or PSM()
q = zeros(1, 7);
robot.fkine(q) % Forward kinematics

%q(3) = 0.075; % Use for PSM to push gripper out of tube
animated_joint = 1; % Joint to animate
joint_min = -pi / 4;
joint_max = pi / 4;
joint_step = pi / 32; % Step size for joint animation
loops = 5; % Number of loops for the animation

for loop = 1:loops
    % Increment the specified joint
    for joint_value = joint_min:joint_step:joint_max
        q(animated_joint) = joint_value;
        robot.plot(q);
        pause(0.25); % Pause for 0.5 seconds to visualize the change
    end

    % Decrement the specified joint
    for joint_value = joint_max:-joint_step:joint_min
        q(animated_joint) = joint_value;
        robot.plot(q);
        pause(0.25); % Pause for 0.5 seconds to visualize the change
    end

end
