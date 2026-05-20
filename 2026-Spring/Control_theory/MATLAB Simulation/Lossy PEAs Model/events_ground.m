function [value, isterminal, direction] = events_ground(t, X)
    
    global g_motor_state

    % calculate lambda
    q = X(1:4);
    q_d = X(6:9);

    [M, B, G, W] = dynamics_mat([q; 0], [q_d; 0]);
    
    if g_motor_state == 1
        u_force = 5;
        lambda = (W*(M\W')) \ (W*(M\(B + G - u_force)));
    else
        lambda = (W*(M\W')) \ (W*(M\(B + G)));
    end
    
    % events
    value(1) = lambda(2);
    isterminal(1) = 1; 
    direction(1) = -1; 

    global g_linear_motor_flag
    global g_motor_state
    global g_stimu_time_
    global g_stimu_time_flag

    if value(1) <= 0 && g_stimu_time_flag == 0 && g_linear_motor_flag == 1
        g_motor_state = 0;
        g_stimu_time_ = [g_stimu_time_ t];  
        g_stimu_time_flag = 1;
    end

end