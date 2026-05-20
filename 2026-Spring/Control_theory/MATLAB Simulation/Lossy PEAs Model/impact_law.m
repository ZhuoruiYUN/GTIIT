function Xnew= impact_law(Xold)

    global mu

    [Mc, ~, ~, W]  = dynamics_mat(Xold(1:5), Xold(6:10));

    Wn = W(2,:);

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
    LambdaHat = LambdaII; %Since CoR's are 0
    
    if abs(LambdaHat(1)) <= mu*LambdaHat(2)
        kappa = 1;
    else
        kappa = mu*LambdaI(2)/(abs(LambdaII(1))-mu*(LambdaII(2)-LambdaI(2)));
    end
    
    Lambda = LambdaI+kappa*(LambdaII-LambdaI);

%     %Check when kappa not = 1 that these are equal 
%     Lambda(1)
%     Lambda(2)*mu*sign(LambdaHat(1))

    Deltaq_d = (Mc\(W'))*Lambda;
    qplus_d = Deltaq_d+Xold(6:10);

    
    Xnew = [x;
            y ;
            theta;
            l;
            phi;
            W*qplus_d; %2 values; x_d and y_d of new stance foot, and 0 by design (in stick)
            -qplus_d(3);
            -qplus_d(4);
            dphi]; 
    
end


