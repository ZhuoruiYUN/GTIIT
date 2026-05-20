function [value,isterminal,direction] = events_flight(t,X_)

    
    y = X_(1);
    l = X_(2);
    
    value(1) = y - l;
    isterminal(1) = 1;
    direction(1) = -1;

end