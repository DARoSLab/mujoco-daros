function tau = controller_arm(z, A, cori, grav)
    global z0
    tau = zeros(3,1);
    
%     qdot = z(9 + 3:end);
%     qdd = A\(tau - cori - grav);

    
    for i = 1:3
        tau(i) = 10*(z0(i) - z(i)) + 1.5*(-z(3 +i));
    end
    tau =  tau + grav;
end