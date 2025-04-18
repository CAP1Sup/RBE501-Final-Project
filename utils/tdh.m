function T = tdh(theta, d, a, alpha)
%tdh Calculates transformation matrix from DH parameters
%   Inputs: theta (radians), d (m), a (m), alpha (radians)

% Preliminary calculations:
c_th = cos(theta);
s_th = sin(theta);
c_al = cos(alpha);
s_al = sin(alpha);

% Formula from Robotics Modeling, Planning, and Control Textbook (2009)
T = [c_th, -s_th*c_al, s_th*s_al, a*c_th;
    s_th, c_th*c_al, -c_th*s_al, a*s_th;
    0, s_al, c_al, d;
    0, 0, 0, 1];

end

