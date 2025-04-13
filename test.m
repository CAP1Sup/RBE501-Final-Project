clear;

robot = MTM();
q = zeros(1, 7);
robot.fkine(q) % Forward kinematics
robot.plot(); % Plot the robot in the given configuration
