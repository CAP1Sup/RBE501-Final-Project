clear, clc, close all
addpath('../utils')

%% Define the Screw Axes
syms L [4,1] real;
w = [0 0 1]';
S = [
    screwaxis(w, [0 0 0]'), ...             % S1
    screwaxis(w, [-L3 0 0]'), ...           % S2
    screwaxis(w, [0 L1 0]'), ...            % S3
    screwaxis(w, [-L3 L1 0]'), ...          % S4
    screwaxis(w, [0 L1+L2 0]'), ...         % S5
    screwaxis(w, [-L3 L1+L2 0]'), ...       % S6
    screwaxis(w, [-L3-L4 L1+L2 0]'), ...    % S7
    screwaxis(w, [-L3-L4 L1 0]')            % S8
];

%% Define Home Configuration Matrices


%% The Forward Kinematics
syms q

f1 = twist2ht(S(:,1), q);
f2 = simplify(twist2ht(S(:,2), q));

f3a = simplify(f1 * twist2ht(S(:,5), -q));
f3b = simplify(f2 * twist2ht(S(:,6), -q));

f4a = simplify(f1 * twist2ht(S(:,3), -q));
f4b = simplify(f2 * twist2ht(S(:,4), -q));

f5a = simplify(f3a * twist2ht(S(:,7), q));
f5b = simplify(f3b * twist2ht(S(:,7), q));
f5c = simplify(f4a * twist2ht(S(:,8), q));
f5d = simplify(f4b * twist2ht(S(:,8), q));