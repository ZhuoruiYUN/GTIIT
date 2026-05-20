function [value,isterminal,direction] = events_flight(t,X)
    
    y = X(2);
    theta = X(3);
    l = X(4);

    
    value(1) = y - l * cos(theta);
    isterminal(1) = 1;
    direction(1) = -1;

    global g_stimu_time_flag
    if value(1) < 0
        g_stimu_time_flag = 0;
    end

end