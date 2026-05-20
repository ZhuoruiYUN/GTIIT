function dXdt = sys_ground(t,X)
    
    qb = X(1:2);
    qb_d = X(4:5);
    
    [qs, qs_d, qs_dd, u_force] = motor_input(t, X, false);
    % qs = 0; qs_d = 0; qs_dd = 0; u_force = 0;

    q = [qb;qs];
    q_d = [qb_d;qs_d];
    
    [q_dd,~] = dyn_sol_ground(q,q_d,u_force);
    q_dd(3) = qs_dd;
    dXdt = [q_d;q_dd];

end