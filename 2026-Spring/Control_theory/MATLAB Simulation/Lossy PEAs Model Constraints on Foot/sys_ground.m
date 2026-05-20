function dXdt = sys_ground(t,X)
    
    q = X(1:5);       
    q_d = X(6:10);    
   

    % determine the state running the motor
    global g_linear_motor_flag
    global g_old_lambda
    global g_motor_state
    global g_stimu_time
    global g_time_lambda 
    global g_lambda 
    global g_uforce
    if g_motor_state == 0 && g_linear_motor_flag == 1

        [~,LAMBDA_N] = dyn_sol_ground(q,q_d,0);
        
        if LAMBDA_N(2) < g_old_lambda
            g_motor_state = 1;
            g_stimu_time = [g_stimu_time t];
        end
        
        g_old_lambda = LAMBDA_N(2);

        u_force = 0;

    else

        u_force = g_uforce;
        g_old_lambda = 0;

    end
    %
    
    [q_dd,~] = dyn_sol_ground(q,q_d,u_force);

    dXdt = [q_d; q_dd];

end