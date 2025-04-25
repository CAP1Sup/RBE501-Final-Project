function q = q_rand(n, min, max)
    %Q_RAND Returns n number of random joint values between a min and max
    %value.

    q = zeros(n, 1);

    for i = 1 : n
        q(i) = (max - min)*rand + min;
    end

end