function dXdt = sys_flight(t,X)


    qb = X(1:4); % [x; y; theta; l]
    qb_d = X(7:10); % [dx; dy; dtheta; dl]
    
    
    u = 0; u_d = 0; u_dd = 0;
    % phi = 0; phi_d = 0; phi_dd = 0;
    [phi, phi_d, phi_dd] = phi_input(t, X, false);


    q = [qb; phi; u];
    q_d = [qb_d; phi_d; u_d];
    
    qs_dd = [phi_dd; u_dd];
    q_dd = dyn_sol_flight(q,q_d,qs_dd);
    
    q_dd(5) = phi_dd;
    q_dd(6) = u_dd;
    
    dXdt = [q_d;q_dd];

end