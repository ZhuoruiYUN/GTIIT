function qb_dd= dyn_sol_flight(q,q_d,qs_dd)
    
    b = 1:4;
    s = 5:6;
    
    [M,B,G,~] = dynamics_mat(q,q_d);
    Mbb = M(b,b);
    Mbs = M(b,s);
    Bb = B(b);
    Gb = G(b);
    
    qb_dd = Mbb\(-Mbs*qs_dd-Bb-Gb);

end