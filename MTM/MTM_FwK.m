clear, clc, close all
addpath('../utils/')
nTests = 10;
plotRobot = true;

%% Screw Axes: 

% Link Lengths:
Lbp = 0.2154;
Lbf = 0.1;
La = 0.2794;
Lf = 0.3645;
H = 0.1056;


% Screw Axes:
%w1 = [0, 1, 0]';
w1 = [0, 0, 1]';
w2 = [0, 1, 0]';
w3 = [0, 1, 0]';
w4 = [0, 0, 1]';
w5 = [0, 1, 0]';
w6 = [0, 0, 1]';
w7 = [1, 0, 0]';
wb = [w1, w2, w3, w4, w5, w6, w7];

p1 = [0, 0, 0]';
p2 = [0, 0, -Lbp]';
p3 = [0, 0, -(Lbp + La)]';
p4 = [-Lf, 0, 0]';
p5 = [-Lf, 0, -(Lbp + La - H)]';
p6 = p4;
p7 = [0, 0, -(Lbp + La - H)]';
pb = [p1, p2, p3, p4, p5, p6, p7];

Sb = revolute_screw(wb, pb);
M = [
    0 0 1 -Lf;
    1 0 0 0;
    0 1 0 -(La + Lbp - H);
    0 0 0 1
];

qlim = [
    -pi/2, pi/2;
    -pi/2, pi/2;
    -pi/2, pi/2;
    -pi/2, pi/2;
    -pi/2, pi/2;
    -pi/2, pi/2;
    -pi/2, pi/2
];
%% Build Robot Model

mtm_robot = SerialLink([RevoluteMDH('a', 0, 'd', -Lbp, 'alpha', 0), ...
                        RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', pi/2), ...
                        RevoluteMDH('a', La, 'd', 0, 'alpha', 0, 'offset', pi/2), ...
                        RevoluteMDH('a', Lf, 'd', H, 'alpha', -pi/2), ...
                        RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2), ...
                        RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', pi/2), ...
                        RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', pi)]);

mtm_robot.teach(zeros(1,7));

%% Test Forward Kinematics

fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

q_list = [];

% Test the forward kinematics for 100 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));

    % Generate a random configuration
    q = randq(qlim);
    q_list = [q_list; q];
     

    % Calculate the forward kinematics
    T = fkine(Sb,M,q, 'space');

    if plotRobot
        %mtm_robot.plot(q, 'movie', 'MTM_Kinematics.mp4');
        title('Forward Kinematics Test');
    end

    assert(all(all(abs(double(mtm_robot.fkine(q)) - T) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');

mtm_robot.plot(q_list, 'movie', 'MTM_Kinematics.mp4', 'fps', 2);