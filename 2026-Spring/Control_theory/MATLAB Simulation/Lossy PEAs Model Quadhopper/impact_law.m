function Xnew= impact_law(Xold)


    [Mc, ~, ~, W]  = dynamics_mat(Xold(1:4), Xold(5:8));

    x = Xold(1);
    y = Xold(2);
    theta = Xold(3);
    l = Xold(4);

    pdot = W*Xold(5:8);

    Ac = (W/Mc*W');
    LambdaI = [0; -pdot(2)/Ac(2,2)];
    LambdaII = -Ac\pdot;

    kappa = 1; % no slip
    Lambda = LambdaI+kappa*(LambdaII-LambdaI);
    Deltaq_d = (Mc\(W'))*Lambda;
    qplus_d = Deltaq_d+Xold(5:8);

    Xnew = [x;
            y ;
            theta;
            l;
            W*qplus_d;  %2 values;
            qplus_d(3);
            qplus_d(4)];
    
end


