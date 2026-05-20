function Xnew= impact_law(Xold)


    [Mc, ~, ~, W]  = dynamics_mat(Xold(1:5), Xold(6:10));

    x = Xold(1);
    y = Xold(2);
    theta = Xold(3);
    l = Xold(4);
    phi = Xold(5);
    dphi = Xold(10);

    pdot = W*Xold(6:10);

    Ac = (W/Mc*W');
    LambdaI = [0; -pdot(2)/Ac(2,2)];
    LambdaII = -Ac\pdot;

    kappa = 1; % no slip
    Lambda = LambdaI+kappa*(LambdaII-LambdaI);
    Deltaq_d = (Mc\(W'))*Lambda;
    qplus_d = Deltaq_d+Xold(6:10);

    Xnew = [x;
            y ;
            theta;
            l;
            phi;
            W*qplus_d; %2 values;
            qplus_d(3);
            qplus_d(4);
            dphi]; 
    
end


