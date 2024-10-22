function tau = controller_biped_toe(z, A, cori, grav)
    global z0
    tau = zeros(11,1);
    
%     qdot = z(9 + 3:end);
%     qdd = A\(tau - cori - grav);

    
    for i = 1:8
        tau(3+i) = 400*(z0(3+i) - z(3+i)) + 1.5*(-z(11 + 3 +i));
    end
end