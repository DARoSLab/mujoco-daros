function dz = dynamics_biped(z, tau)    
    % Get mass matrix
    A = A_biped(z);
    cori = coriolis_biped(z);
    grav = grav_biped(z);
    
    % Compute Controls
    qdd = A\(tau - cori - grav);
    
    dz = 0*z;
    dim = length(dz);
    % Form dz
    dz(1:dim/2) = z(dim/2+1:dim);
    dz(dim/2+1:dim) = qdd;
end