function dXdt = sys_flight(t,X)
    

    qb = X(1:4);       % [x; y; theta; l]
    qb_d = X(6:9);     % [dx; dy; dtheta; dl]

    % calculate velocity of m1
    theta = X(3);
    l = X(4);
    dx = X(6);
    dl = X(9);
    dtheta = X(8);
    vx_m1 = (dx + dl*sin(theta) + dtheta*l*cos(theta));
    %
    
    global g_wheel_motor_flag
    
    if g_wheel_motor_flag == 1

        theta_des = raibertHopperCtrl(vx_m1, l, 1);
        theta_d_des = 0;

        [phi_d, phi_dd] = phi_control(t, X, theta_des, theta_d_des);
        phi = X(5); 

    else
        phi = 0; phi_d = 0; phi_dd = 0;
    end
    

    q = [qb; phi];
    q_d = [qb_d; phi_d];

    q_dd = dyn_sol_flight(q,q_d,phi_dd);
    
    q_dd = [q_dd; phi_dd];

    dXdt = [q_d; q_dd];
    
end