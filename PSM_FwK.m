% RBE 501: Robot Dynamics
% Authors: Callahan Henry (and group)
% Date: 4/18/25
% Description: This script compares the forward kinematics using the 
    % provided DH method and our PoE calculations


% RBE 501 - Robot Dynamics - Spring 2025
% Homework 2, Problem 1
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/06/2025
clear, clc, close all
addpath('utils');

plotOn = true;
nTests = 30; % number of random test configurations

%% Create the manipulator
% Link length values (meters)
d3 = 0.516;
d5 = 0.0156;
d6 = d5 - 0.00091;

l_RCC = 0.4318; % meters
l_tool = 0.4162;


robot = SerialLink([RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', pi/2), ...
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2), ...
                    PrismaticMDH('a', d3,'alpha', pi/2, 'qlim', [0, 0.416 + 0.25419], 'theta', 0, 'offset', -l_RCC),...
                    RevoluteMDH('a', 0, 'd', l_tool, 'alpha', 0, 'offset', 0),...
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2), ...
                    RevoluteMDH('a', 0.00091, 'd', 0, 'alpha', -pi/2, 'offset', pi/2)...
                    ], ...
                    'name', 'PSM');


% Joint limits
% qlim = [-pi/2  pi/2;  % q(1)
%         -pi/4  pi/2;  % q(2)
%         -pi/12 pi/3]; % q(3)

% Display the manipulator in the home configuration
q = zeros(1, 6);
% q(1, 3) = 0.416;
robot.teach(q);

%% Part A - Calculate the screw axes
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints

S = [0, -1, 0, 0, 0, 0;
    -1, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, -1;
    0, 0, -1, -d3, 0, 0;
    -1, 0, 0, 0, -d5, d3;
    0, -1, 0, d6, 0, 0]';

% %% Part B - Calculate the forward kinematics with the Product of Exponentials formula
% % First, let us calculate the homogeneous transformation matrix M for the
% % home configuration
% 
% M = [1, 0, 0, 0;
%      0, 0, -1, d3;
%      0, 1, 0, d6;
%      0, 0, 0, 1];
% 
% fprintf('---------------------Forward Kinematics Test---------------------\n');
% fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
% fprintf('Progress: ');
% nbytes = fprintf('0%%');
% 
% % Test the forward kinematics for 100 random sets of joint variables
% for ii = 1 : nTests
%     fprintf(repmat('\b',1,nbytes));
%     nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
% 
%     % Generate a random configuration
%     q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
%          qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
%          qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand()];
% 
%     % Calculate the forward kinematics
%     T = fkine(S,M,q);
% 
%     if plotOn
%         robot.teach(q);
%         title('Forward Kinematics Test');
%     end
% 
%     assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
% end
% 
% fprintf('\nTest passed successfully.\n');
% 
% 
% %% Part C - Calculate the Space Jacobian of the manipulator
% fprintf('-------------------Differential Kinematics Test------------------\n');
% fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
% fprintf('Progress: ');
% nbytes = fprintf('0%%');
% 
% % Test the correctness of the Jacobian for 100 random sets of joint
% % variables
% for ii = 1 : nTests
%     fprintf(repmat('\b',1,nbytes));
%     nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
% 
%     % Generate a random configuration
%     q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
%          qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
%          qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand()];
% 
%     % Calculate the Forward Kinematics
%     T = fkine(S,M,q);
% 
%     % Calculate the Jacobian
%     J = jacob0(S,q);
% 
%     if plotOn
%         robot.teach(q);
%         title('Differential Kinematics Test');
%     end
% 
%     % Test the correctness of the Jacobian
%     Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
%     assert(all(all(abs(double(robot.jacob0(q)) - Jcoords) < 1e-10)));
% end
% 
% fprintf('\nTest passed successfully.\n');
% 
% %% Part D - Inverse Kinematics
% fprintf('----------------------Inverse Kinematics Test--------------------\n');
% fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
% fprintf('Progress: ');
% nbytes = fprintf('0%%');
% 
% % Calculate the twist representing the robot's home pose
% currentPose = MatrixLog6(M);
% currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';
% 
% % Set the current joint variables
% currentQ = zeros(1,3);
% 
% if plotOn
%     robot.teach(currentQ);
%     h = triad('matrix', M, 'tag', 'Target Pose', 'linewidth', 2.5, 'scale', 0.5);
% end
% 
% % Generate the test configurations
% q = [linspace(0,pi/2,nTests);
%      linspace(0,pi/6,nTests);
%      linspace(0,pi/6,nTests)];
% 
% for ii = 1 : nTests
%     fprintf(repmat('\b',1,nbytes));
%     nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
% 
%     % Generate the robot's pose
%     T = fkine(S,M,q(:,ii)');
%     targetPose = MatrixLog6(T);
%     targetPose = [targetPose(3,2) targetPose(1,3) targetPose(2,1) targetPose(1:3,4)']';
% 
%     if plotOn
%         set(h, 'matrix', T);
%         title('Inverse Kinematics Test');
%         drawnow;
%     end
% 
%     % Inverse Kinematics
%     while norm(targetPose - currentPose) > 1e-3
%         % Implementing Newton-Raphson Method for IK
%         deltaQ = pinv(J) * (targetPose - currentPose);
% 
%         currentQ = currentQ + deltaQ';
% 
%         T = fkine(S,M,currentQ);
%         currentPose = MatrixLog6(T);
%         currentPose = [currentPose(3,2) ...
%                        currentPose(1,3) ...
%                        currentPose(2,1) ...
%                        currentPose(1:3,4)']';
% 
%         if plotOn
%             try
%                 robot.teach(currentQ);
%                 drawnow;
%             catch e
%                 continue;
%             end
%         end
%     end
% end
% 
% fprintf('\nTest passed successfully.\n');
% 
% 
% %% Helper Functions
% 
% % fkine  Calculates the fwk using the product of exponentials
% %   T = fkine(S,M,q) calculates the forward kinematics using screw axes S,
% %   home configuration M, and joint values q
% function T = fkine(S,M,q)
%     % Loop through the multiplication all transformation matrices from twists
%     T = eye(4);
%     for i = 1:length(q)
%         T = T * twist2ht(S(:,i), q(i));
%     end
%     % Multiply transformation matrices with M representing the home configuration to get fwk
%     T = T * M;
% end
% 
% 
% 
% 
