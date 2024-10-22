function [pt, vel] = fn_bezier_pt(alpha, s, sdot)
    M = length(alpha)-1;
    coeff = alpha;
    coeff_vel = zeros(M-1,1);
    for k = 0:M
        coeff(k+1) = alpha(k+1) * factorial(M)/(factorial(k) * factorial(M-k));
        if k < M
            coeff_vel(k+1) = (alpha(k+2) - alpha(k+1)) * factorial(M)/(factorial(k) * factorial(M-k-1));
        end
    end
    pt = 0;
    vel = 0;
    for k = 0:M
        pt = pt + coeff(k+1) * s^k * (1-s)^(M-k);
        if k < M
            vel = vel + coeff_vel(k+1) * s^k * (1-s)^(M-k-1) * sdot;
        end
    end
end