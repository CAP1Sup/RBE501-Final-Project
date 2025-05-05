function q = randq(qlim)
    % RANDQ Generates a random robot configuration within the joint limits
    % provided by QLIM.
    %
    %   Input:
    %       QLIM - An Nx2 matrix where the first column is the joint
    %       minimums and the second column is the maximums. The Nth row
    %       represents the Nth joint.
    %
    %   Output:
    %       Q - The random joint configuration as a vector with N entries.

    % Initialzie the q vector
    q = zeros(1, size(qlim,1));

    % Generate each random joint position.
    for i = 1:size(q,2)
        q(i) = qlim(i,1) + (qlim(i,2) - qlim(i,1))* rand();
    end
end
