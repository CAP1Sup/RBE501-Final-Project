function traj = make_trajectory(type, params)
% MAKE_TRAJECTORY Creates cubic or quintic trajectories.
%   INPUTS: 
%       type - a string indicating the type of trajectory that you want to generate.
%                Acceptable values: {'cubic' | 'quintic'}
%       params - a structure containing the prescribed trajectory parameters
%           params.t  - 2-vector prescribing the initial and final time
%           params.dt - desired time step
%           params.q - 2-vector prescribing the starting and final positions
%           params.v - 2-vector describing the starting and final velocities
%           params.a - 2-vector describing the starting and final accelerations (only for quintic polynomials)
%   OUTPUTS:
%       traj - a structure containing the trajectory
%           traj.t - n-vector representing time
%           traj.q - n-vector representing position over time
%           traj.v - n-vector representing velocity over time
%           traj.a - n-vector representing acceleration over time

% Initializing Variables
t0 = params.t(1);
tf = params.t(2);
q0 = params.q(1);
qf = params.q(2);
v0 = params.v(1);
vf = params.v(2);


% Creating Time Vector:
t = t0 : params.dt : tf;


switch upper(type)
    case 'CUBIC'
        % Solving equation for Coefficients:
        cubic_eq = [1, t0, t0^2, t0^3;
                    0, 1, 2*t0, 3*(t0^2);
                    1, tf, tf^2, tf^3;
                    0, 1, 2*tf, 3*(tf^2);];
        conditions = [q0, v0, qf, vf]';
        a_coefs = cubic_eq \ conditions;
        
        % Finding the trajectory:
        q = a_coefs(1) + a_coefs(2)*t + a_coefs(3)*t.^2 + a_coefs(4)*t.^3;
        v = a_coefs(2) + 2*a_coefs(3)*t + 3*a_coefs(4)*t.^2;
        a = 2*a_coefs(3) + 6*a_coefs(4)*t;
        
        % Saving Results
        traj.t = t;
        traj.q = q;
        traj.v = v;
        traj.a = a;
       

    case 'QUINTIC'
        % Defining Additional Constants
        a0 = params.a(1);
        af = params.a(2);

        % Solving equation for Coefficients:
        quintic_eq = [1, t0, t0^2, t0^3, t0^4, t0^5;
                    0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;
                    0, 0, 2, 6*t0, 12*(t0^2), 20*t0^3;
                    1, tf, tf^2, tf^3, tf^4, tf^5;
                    0, 1, 2*tf, 3*(tf^2), 4*(tf^3), 5*(tf^4);
                    0, 0, 2, 6*tf, 12*(tf^2), 20*(tf^3);];
        conditions = [q0, v0, a0, qf, vf, af]';
        a_coefs = quintic_eq \ conditions;
        
        % Finding the trajectory:
        q = a_coefs(1) + a_coefs(2)*t + a_coefs(3)*t.^2 + a_coefs(4)*t.^3 + a_coefs(5)*t.^4 + a_coefs(6)*t.^5;
        v = a_coefs(2) + 2*a_coefs(3)*t + 3*a_coefs(4)*t.^2 + 4*a_coefs(5)*t.^3 + 5*a_coefs(6)*t.^4;
        a = 2*a_coefs(3) + 6*a_coefs(4)*t + 12*a_coefs(5)*t.^2 + 20*a_coefs(6)*t.^3;
        
        % Saving Results
        traj.t = t;
        traj.q = q;
        traj.v = v;
        traj.a = a;
end

end
