function [value, isterminal, direction] = events_ground(t, X)
    % shared the force
    persistent last_u_force;
    
    % avoid calcualte the force repetitively
    if isempty(last_u_force)
        [~, ~, ~, last_u_force] = motor_input(t, X, false);
    end
    
    % calculate lambda
    q = X(1:3);
    q_d = X(4:6);
    [M, B, G, wn] = dynamics_mat(q, q_d);
    lambda = (wn*(M\wn')) \ (wn*(M\(B + G - last_u_force)));
    
    % events
    value = lambda;
    isterminal = 1; 
    direction = -1; 
end