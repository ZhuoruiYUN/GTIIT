function dXdt = sys_ground(t,X)
    
    qb = X(1:4); % [x; y; theta; l]
    qb_d = X(7:10); % [dx; dy; dtheta; dl]
    
    % [u, u_d, u_dd, u_force] = u_input(t, X, false);
    u = 0; u_d = 0; u_dd = 0; u_force = 0;
    phi = 0; phi_d = 0; phi_dd = 0;
    

    q = [qb; phi; u];
    q_d = [qb_d; phi_d; u_d];

    [q_dd,~] = dyn_sol_ground(q,q_d,u_force);
    
    q_dd(5) = phi_dd;
    q_dd(6) = u_dd;
    
    dXdt = [q_d;q_dd];

end