function adV = ad(V)
    %AD Calculates the ad matrix of a twist V, so that we can later
    %calculate the lie bracket between V and another twist
    %   INPUTS:
    %       V: 6x1 twist
    %   OUTPUTS: 
    %       adV = ad Matrix (6x6 matrix)

    % Extracting Componets from Twist:
    w = V(1:3); 
    v = V(4:6);

    % Computing Skew Matrices:
    w_skew = skew(w);
    v_skew = skew(v);

    % Putting it together to form [adv]:
    adV = [w_skew, zeros(3);
           v_skew, w_skew];
end