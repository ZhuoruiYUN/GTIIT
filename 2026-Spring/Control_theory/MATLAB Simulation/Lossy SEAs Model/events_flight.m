function [value,isterminal,direction] = events_flight(t,X)

    y = X(2);
    th = X(3);
    l = X(4);

    value(1) = y - l * cos(th);
    isterminal(1) = 1;
    direction(1) = -1;

end