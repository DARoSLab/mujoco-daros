function tau = controller_biped_toe(z)
    global z0 
    dim = length(z0);
    dim_q = dim/2;
    dim_act = dim_q - 3;
    
    tau = zeros(dim_q,1);
    
    for i = 1:dim_act
        tau(3+i) = 300*(z0(3+i) - z(3+i)) + 2.5*(-z(dim_q + 3 +i));
    end
end