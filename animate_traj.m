clear;
addpath("./utils");

increment = 10;

robot = MTM();
[q, qd, qdd, meas_tau, est_tau] = load_traj('./traj/mtm/train', 'est_tau_without_cable.csv');
video_time = 20; % seconds

%robot = PSM();
%[q, qd, qdd, meas_tau, est_tau] = load_traj('./traj/psm/train');
%video_time = 30; % seconds

disp("Joint positions loaded, simulating trajectory...");

writerObj = VideoWriter('test.mp4', 'MPEG-4');
writerObj.FrameRate = (size(q, 1) / increment) / video_time;
open(writerObj);

fprintf("Simulating trajectory...");
nbytes = fprintf('0%%');

for ii = 1:increment:size(q, 1)
    fprintf(repmat('\b', 1, nbytes));
    nbytes = fprintf('%3.0f%%', 100 * (ii / (size(q, 1) - 1)));

    robot.plot(q(ii, :));

    writeVideo(writerObj, getframe(gcf));
end

close(writerObj);

fprintf("\nTrajectory simulation complete.");
