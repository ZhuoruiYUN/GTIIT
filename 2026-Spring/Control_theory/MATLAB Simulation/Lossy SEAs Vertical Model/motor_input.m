function [u, u_dot, u_ddot, u_force] = motor_input(t, X, reset_flag)
    % input:
    %   t - current time
    %   X - cooordinates [y; l; u; dy; dl; du]
    % output:
    %   u - position
    %   u_dot - velocity
    %   u_ddot - acc.
    %   u_force - force
    
    persistent last_u_force_shared;
    persistent is_extending extend_start_time last_phase;
    
    % init
    if reset_flag 
        is_extending = false;
        extend_start_time = Inf;
        last_phase = "idle";
    end

    if isempty(is_extending)
        is_extending = false;
        extend_start_time = Inf;
        last_phase = "idle";
    end
    
    % parameters setting
    trigger_height = 0.4;      % trigger height (m)
    speed_threshold = 0.1;    % velocity threshold (m/s)
    max_u = 0.04;               % max position (m)
    max_force = 1;            % max force (N)
    ramp_time = 0.1;           % acc./dec. time (s)
    hold_time = 0.1;           % hold time (s)
    total_duration = 2*ramp_time + hold_time; % 总动作时间
    
    % current states
    y = X(1);
    dy = X(4);
    
    % detect the situations
    if ~is_extending && y < trigger_height && abs(dy) < speed_threshold
        is_extending = true;
        extend_start_time = t;
        last_phase = "accelerating";
        % fprintf('t=%.3f: start u motion\n', t);
    end
    
   
    if is_extending
        elapsed = t - extend_start_time;
        
        if elapsed <= ramp_time
            % acc. (0 → max_u)
            progress = elapsed / ramp_time;
            u = max_u * (1 - cos(pi*progress))/2;  % smooth the position
            u_dot = max_u * pi/(2*ramp_time) * sin(pi*progress);
            u_ddot = max_u * pi^2/(2*ramp_time^2) * cos(pi*progress);
            u_force = max_force;
            last_phase = "accelerating";
            
        elseif elapsed <= ramp_time + hold_time
            % holding
            u = max_u;
            u_dot = 0;
            u_ddot = 0;
            u_force = 0;
            last_phase = "holding";
            
        elseif elapsed <= total_duration
            % dec. (max_u → 0)
            progress = (elapsed - ramp_time - hold_time) / ramp_time;
            u = max_u * (1 + cos(pi*progress))/2;  
            u_dot = -max_u * pi/(2*ramp_time) * sin(pi*progress);
            u_ddot = -max_u * pi^2/(2*ramp_time^2) * cos(pi*progress);
            u_force = -max_force;  
            last_phase = "decelerating";
            
        else
            
            u = 0;
            u_dot = 0;
            u_ddot = 0;
            u_force = 0;
            is_extending = false;
            % fprintf('t=%.3f: finish u motion\n', t);
        end
    else
        
        u = 0;
        u_dot = 0;
        u_ddot = 0;
        u_force = 0;
    end

    last_u_force_shared = u_force;
end