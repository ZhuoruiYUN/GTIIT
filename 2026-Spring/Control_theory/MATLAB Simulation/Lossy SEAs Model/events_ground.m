function [value, isterminal, direction] = events_ground(t, X)

    % % shared the u force
    % persistent last_u_force;
    % 
    % % avoid calcualte the force repetitively
    % if isempty(last_u_force)
    %     [~, ~, ~, last_u_force] = u_input(t, X, false);
    % end
    last_u_force = 0;


    % calculate lambda
    q = X(1:6);
    q_d = X(7:12);
    
    [~, LAMBDA] = dyn_sol_ground(q,q_d,last_u_force);
    
    % events
    value = LAMBDA(2);
    isterminal = 1; 
    direction = -1; 
end