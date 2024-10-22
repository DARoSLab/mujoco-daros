function curve = fn_bezier_curve(alpha, num_pts)
    s = linspace(0, 1, num_pts);
    M = length(alpha)-1;
    curve = s;
    coeff = alpha;
    for k = 0:M
        coeff(k+1) = alpha(k+1) * factorial(M)/(factorial(k) * factorial(M-k));
    end
    for i = 1:num_pts
        curve(i) = 0;
        for k = 0:M
            curve(i) = curve(i) + coeff(k+1) * s(i)^k * (1-s(i))^(M-k);
        end
    end
end

