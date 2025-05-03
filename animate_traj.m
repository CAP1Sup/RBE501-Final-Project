clear;

robot = MTM();
[q, qd, qdd, tau] = load_traj('./traj/mtm/train', 'tau_without_cable.csv');

%robot = PSM();
%[q, qd, qdd, tau] = load_traj('./traj/psm/train');

disp("Joint positions loaded, simulating trajectory...");

for ii = 1:10:size(q, 1)
    robot.plot(q(ii, :));
    pause(0.01);
end

disp("Trajectory simulation complete.");
