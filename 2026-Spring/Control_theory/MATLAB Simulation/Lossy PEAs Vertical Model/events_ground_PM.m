function [value, isterminal, direction] = events_ground(t, X)
    
    global g_motor_state

    % calculate lambda
    q = X(1:2);
    q_d = X(3:4);
    [M, B, G, wn] = dynamics_mat(q, q_d);
    
    if g_motor_state == 1
        u_force = 5;
        lambda = (wn*(M\wn')) \ (wn*(M\(B + G - u_force)));
    else
        lambda = (wn*(M\wn')) \ (wn*(M\(B + G)));
    end
    
    % events
    value(1) = lambda;
    isterminal(1) = 1; 
    direction(1) = -1; 

    dy = X(3);
    value(2) = dy;
    isterminal(2) = 1; 
    direction(2) = 1; 

    global g_motor_state
    global g_stimu_time_
    global g_stimu_time_flag

    if value(1) <= 0 && g_stimu_time_flag == 0
        g_motor_state = 0;
        g_stimu_time_ = [g_stimu_time_ t];  
        g_stimu_time_flag = 1;
    end

end