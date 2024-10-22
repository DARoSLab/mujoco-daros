function tau = controller_biped(z, A, cori, grav)
    global z0
    tau = zeros(9,1);
    
%     qdot = z(9 + 3:end);
%     qdd = A\(tau - cori - grav);

    
    for i = 1:6
        tau(3+i) = 400*(z0(3+i) - z(3+i)) + 3.5*(-z(9 + 3 +i));
    end
end