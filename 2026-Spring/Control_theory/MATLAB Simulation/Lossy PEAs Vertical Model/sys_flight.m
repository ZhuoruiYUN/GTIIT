function dXdt = sys_flight(t,X)

    qb = X(1:2);
    qb_d = X(3:4);

    u_force = 0;

    q = qb;
    q_d = qb_d;
    q_dd = dyn_sol_flight(q,q_d,u_force);
    
    dXdt = [q_d; q_dd];
    
    
end