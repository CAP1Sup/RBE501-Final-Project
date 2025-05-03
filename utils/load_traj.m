function [q, qd, qdd, tau] = load_traj(traj_folder, tau_filename)
    % LOAD_TRAJ Load trajectory data from specified folder.
    %
    %   [q, qd, qdd, tau] = LOAD_TRAJ(traj_folder, tau_filename) loads the
    %   trajectory data from the specified folder. The function reads the
    %   joint positions (q), joint velocities (qd), joint accelerations (qdd),
    %   and joint torques (tau) from CSV files located in the given folder.
    %
    %   INPUTS:
    %       traj_folder  - (string) Path to the folder containing the trajectory
    %                      data files ('q.csv', 'qd.csv', 'qdd.csv').
    %       tau_filename - (string, optional) Name of the CSV file containing
    %                      the joint torques. Defaults to 'tau.csv' if not
    %                      provided.
    %
    %   OUTPUTS:
    %       q   - (matrix) Joint positions loaded from 'q.csv'.
    %       qd  - (matrix) Joint velocities loaded from 'qd.csv'.
    %       qdd - (matrix) Joint accelerations loaded from 'qdd.csv'.
    %       tau - (matrix) Joint torques loaded from the specified tau file.

    % Assume that the name of the tau file is just "tau.csv"
    if ~exist('tau_filename', 'var')
        tau_filename = 'tau.csv';
    end

    % Load the trajectory data
    q = load(fullfile(traj_folder, 'q.csv'));
    qd = load(fullfile(traj_folder, 'qd.csv'));
    qdd = load(fullfile(traj_folder, 'qdd.csv'));
    tau = load(fullfile(traj_folder, tau_filename));
end
