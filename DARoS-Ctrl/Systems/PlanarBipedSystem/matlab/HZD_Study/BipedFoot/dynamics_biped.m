function dz = dynamics_biped(z)    
    % Get mass matrix
    A = A_biped(z);
    cori = coriolis_biped(z);
    grav = grav_biped(z);
    
    % Compute Controls
    tau = controller_biped(z, A, cori, grav);

    
    % Stable joint PD.
    stable_pd = false;
    
    if(stable_pd)
        global z0 dt
        stable_pd = zeros(9,1);
        Kp = 400; Kd = 3.0;
        for i = 1:6
            stable_pd(3+i) = Kp*(z0(3+i) - (z(3+i) + z(9+3+i)*dt)) + Kd*(-z(9 + 3 +i));
        end
        qdd = (A + Kd*dt)\(- cori - grav + stable_pd);
    else
        qdd = A\(tau - cori - grav);
    end
    
    dz = 0*z;
    dim = length(dz);
    % Form dz
    dz(1:dim/2) = z(dim/2+1:dim);
    dz(dim/2+1:dim) = qdd;
end