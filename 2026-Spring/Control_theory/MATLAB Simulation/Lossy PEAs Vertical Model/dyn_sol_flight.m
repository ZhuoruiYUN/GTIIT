function qb_dd= dyn_sol_flight(q,q_d,u_force)

    b = 1:2;
    
    [M,B,G,~] = dynamics_mat(q,q_d);
    Mbb = M(b,b);
    Bb = B(b);
    Gb = G(b);
 
    F = [0;
         u_force];

    qb_dd = Mbb\(F-Bb-Gb);
    
end