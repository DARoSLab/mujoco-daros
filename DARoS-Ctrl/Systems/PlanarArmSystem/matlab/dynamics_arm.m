function dz = dynamics_arm(z)    
  % include directory
    % Get mass matrix
    A = A_arm(z);
    cori = coriolis_arm(z);
    grav = grav_arm(z);
    % Compute Controls
    tau = controller_arm(z, A, cori, grav);
    
%     invA = inv(A);
%     qdd = invA*(tau - cori - grav);
    qdd = A\(tau - cori - grav);
    
    dz = 0*z;
    dim = length(dz);
    % Form dz
    dz(1:dim/2) = z(dim/2+1:dim);
    dz(dim/2+1:dim) = qdd;
end
