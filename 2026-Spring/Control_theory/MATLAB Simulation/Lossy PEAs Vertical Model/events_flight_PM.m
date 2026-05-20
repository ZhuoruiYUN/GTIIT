function [value,isterminal,direction] = events_flight_PM(t,X_)
    
    y = X_(1);
    l = X_(2);
    dy = X_(3);
    
    value(1) = y - l;
    isterminal(1) = 1;
    direction(1) = -1;

    value(2) = dy;
    isterminal(2) = 1;
    direction(2) = -1;

    global g_stimu_time_flag
    if value(1) < 0
        g_stimu_time_flag = 0;
    end

end