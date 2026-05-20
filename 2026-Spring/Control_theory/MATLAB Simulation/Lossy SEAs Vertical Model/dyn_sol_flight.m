function qb_dd= dyn_sol_flight(q,q_d,qs_dd,u_force)
    
    b = 1:2;
    s = 3;
    
    [M,B,G,~] = dynamics_mat(q,q_d);
    Mbb = M(b,b);
    Mbs = M(b,s);
    Mss = M(s,s);
    Bb = B(b);
    Bs = B(s);
    Gb = G(b);
    Gs = G(s);

    qb_dd = Mbb\(u_force-Mbs*qs_dd-Bb-Gb);
    
end