function [q, qd, qdd, meas_tau, est_tau] = load_traj(traj_folder, est_tau_filename)
    % LOAD_TRAJ Load trajectory data from specified folder.
    %
    %   [q, qd, qdd, tau] = LOAD_TRAJ(traj_folder, est_tau_filename) loads the
    %   trajectory data from the specified folder. The function reads the
    %   joint positions (q), joint velocities (qd), joint accelerations (qdd),
    %   measured joint torques (meas_tau), and estimated joint torques (est_tau)
    %   from CSV files located in the given folder.
    %
    %   INPUTS:
    %       traj_folder      - (string) Path to the folder containing the trajectory
    %                          data files ('q.csv', 'qd.csv', 'qdd.csv', 'meas_tau.csv').
    %       est_tau_filename - (string, optional) Name of the CSV file containing
    %                          the joint torques. Defaults to 'est_tau.csv' if not
    %                          provided.
    %
    %   OUTPUTS:
    %       q        - (matrix) Joint positions loaded from 'q.csv'.
    %       qd       - (matrix) Joint velocities loaded from 'qd.csv'.
    %       qdd      - (matrix) Joint accelerations loaded from 'qdd.csv'.
    %       meas_tau - (matrix) Joint torque measurements loaded from 'meas_tau.csv'.
    %       est_tau  - (matrix) Joint torque estimates loaded from the specified est tau file.

    % Assume that the name of the tau file is just "est_tau.csv"
    if ~exist('est_tau_filename', 'var')
        est_tau_filename = 'est_tau.csv';
    end

    % Load the trajectory data
    q = load(fullfile(traj_folder, 'q.csv'));
    qd = load(fullfile(traj_folder, 'qd.csv'));
    qdd = load(fullfile(traj_folder, 'qdd.csv'));
    meas_tau = load(fullfile(traj_folder, 'meas_tau.csv'));
    est_tau = load(fullfile(traj_folder, est_tau_filename));
end
