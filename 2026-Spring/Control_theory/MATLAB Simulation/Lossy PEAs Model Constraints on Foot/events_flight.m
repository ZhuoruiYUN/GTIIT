function [value,isterminal,direction] = events_flight(t,X)
    
    
    value(1) = X(2); % landing
    isterminal(1) = 1;
    direction(1) = -1;

    global g_stimu_time_flag
    if value(1) < 0
        g_stimu_time_flag = 0;
    end

end