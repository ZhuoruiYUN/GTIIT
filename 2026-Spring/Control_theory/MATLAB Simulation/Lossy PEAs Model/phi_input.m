function [phi, phi_dot, phi_ddot] = phi_input(t, X, reset_flag)
    % input:
    %   t - current time
    %   X - cooordinates 
    % output:
    %   phi - position
    %   phi_dot - velocity
    %   phi_ddot - acc.
    %   phi_force - force

    persistent last_phi_force_shared;
    persistent is_extending_phi extend_start_time_phi last_phase_phi;
    
    % init
    if reset_flag 
        is_extending_phi = false;
        extend_start_time_phi = Inf;
        last_phase_phi = "idle";
    end

    if isempty(is_extending_phi)
        is_extending_phi = false;
        extend_start_time_phi = Inf;
        last_phase_phi = "idle";
    end
    
    % parameters setting
    trigger_height = 1.2;      % trigger height (m)
    speed_threshold = 1;    % velocity threshold (m/s)
    max_u = pi/6;               % max position (degrees)
    max_force = 1;            % max torque (N.m)
    ramp_time = 0.1;           % acc./dec. time (s)
    hold_time = 0.1;           % hold time (s)
    total_duration = 2*ramp_time + hold_time; % 总动作时间
    
    % current states
    y = X(2);
    dy = X(7);
    
    % detect the situations
    if ~is_extending_phi && y > trigger_height && (dy - speed_threshold) > 0
        
        is_extending_phi = true;
        extend_start_time_phi = t;
        last_phase_phi = "accelerating";
        % fprintf('t=%.3f: start phi motion\n', t);
    end
    
   
    if is_extending_phi 
        elapsed = t - extend_start_time_phi;
        
        if elapsed <= ramp_time
            % acc. (0 → max_u)
            progress = elapsed / ramp_time;
            phi = max_u * (1 - cos(pi*progress))/2;  % smooth the position
            phi_dot = max_u * pi/(2*ramp_time) * sin(pi*progress);
            phi_ddot = max_u * pi^2/(2*ramp_time^2) * cos(pi*progress);
            phi_force = max_force;
            last_phase_phi = "accelerating";
            
        elseif elapsed <= ramp_time + hold_time
            % holding
            phi = max_u;
            phi_dot = 0;
            phi_ddot = 0;
            phi_force = 0;
            last_phase_phi = "holding";
            
        elseif elapsed <= total_duration
            % dec. (max_u → 0)
            progress = (elapsed - ramp_time - hold_time) / ramp_time;
            phi = max_u * (1 + cos(pi*progress))/2;  
            phi_dot = -max_u * pi/(2*ramp_time) * sin(pi*progress);
            phi_ddot = -max_u * pi^2/(2*ramp_time^2) * cos(pi*progress);
            phi_force = -max_force;  
            last_phase_phi = "decelerating";
            
        else
            
            phi = 0;
            phi_dot = 0;
            phi_ddot = 0;
            phi_force = 0;
            is_extending_phi = false;
            % fprintf('t=%.3f: finish phi motion\n', t);
        end
    else
        
        phi = 0;
        phi_dot = 0;
        phi_ddot = 0;
        phi_force = 0;
    end

    last_phi_force_shared = phi_force;
end 