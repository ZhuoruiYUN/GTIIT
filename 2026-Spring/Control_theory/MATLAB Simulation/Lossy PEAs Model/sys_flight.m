function dXdt = sys_flight(t,X)
    

    qb = X(1:4);       % [x; y; theta; l]
    qb_d = X(6:9);     % [dx; dy; dtheta; dl]

    global g_wheel_motor_flag
    
    if g_wheel_motor_flag == 1
        [phi, phi_d, phi_dd] = phi_input(t, X, false);
    else
        phi = 0; phi_d = 0; phi_dd = 0;
    end

    
    q = [qb; phi];
    q_d = [qb_d; phi_d];

    q_dd = dyn_sol_flight(q,q_d,phi_dd);
    
    q_dd = [q_dd; phi_dd];

    dXdt = [q_d; q_dd];
    
end