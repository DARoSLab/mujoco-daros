function tau = controller_biped(z)
    global z0
    dim = length(z0);
    tau = zeros(dim/2,1);

    for i = 1:4
        tau(3+i) = 400*(z0(3+i) - z(3+i)) + 3.5*(-z(dim/2 + 3 +i));
    end
end