classdef SLIPdynamics
    
    properties  

        g = 9.81; % Gravity (m/s^2)
        M = 10; % Body Mass (kg)
        L0 = 0.7; % Resting leg lenth (m)
        K = 2000; % Leg Spring Constant (N/m)
        dataTimeStep = 0.01;  % time step between data points (sec)
        dynamic_state = 0; % State: 0 = flight, 1 = Stance
        phi_td = 0; % Desired Touchdown Leg Angle
        t = []; % Time
        dynamic_state_arr = 0; % Recorded State
        
        % Body Position (m)
        x_body = 0;
        y_body = 1;

        % Body Velocity (m/s)
        dx_body = 0;
        dy_body = 0;

        % Leg Angle (foot to body from vertical) (rads)
        phi = 0;

        % Leg Length
        L = 0.7;
        
        % foot x position (m)
        x_foot = 0;

    end
    
    methods
        
        function o = simulate(o,controller,tspan,des_vel)
        % simulate Run a simulation with specified controller
        
           o.t = tspan(1);
           while o.t(end) < max(tspan)
            switch o.dynamic_state
                case 0 % Flight
                    dyn = @(t,y) [y(2);
                                  0;
                                  y(4);
                                  -o.g];
                              
                    options = odeset('Events',@(t,y) (flightTransitions(t,y,o))); 
                    
                    y0 = [o.x_body(end);o.dx_body(end);o.y_body(end);o.dy_body(end)]; % state from stance
                    
                case 1 % Stance
                    dyn = @(t,y) [y(2);
                                  y(1)*o.K*(o.L0-sqrt(y(1)^2 + y(3)^2))/(sqrt(y(1)^2 + y(3)^2)*o.M);
                                  y(4);
                                  y(3)*o.K*(o.L0-sqrt(y(1)^2 + y(3)^2))/(sqrt(y(1)^2 + y(3)^2)*o.M)-o.g];
                              
                    options = odeset('Events',@(t,y) (stanceTransitions(t,y,o)));

                    y0 = [o.x_body(end) - o.x_foot(end);o.dx_body(end);o.y_body(end);o.dy_body(end)]; % state from flight
            
            end 
             
            % Adjust to pick up next simulation at where the last one ended
            tspan = o.t(end):o.dataTimeStep:tspan(end);
                
            [t_out,y_out,~,~,~] = ode45(dyn,tspan,y0,options);
              
            o = fillSimData(o,t_out,y_out);
            o = switchMode(o);
            o = controller(o,des_vel); % update leg touchdown angle
           end 
            
        end 

    end
    
end 

function o = fillSimData(o,t,y)
% This function copies the state data (y) into the object members. 
% What the state variables represent is different depending on the
% dynamic state (stance, flight).

    o.dynamic_state_arr = [o.dynamic_state_arr;o.dynamic_state*ones(size(t))];
    switch o.dynamic_state
        case 0 % Flight phase
            o.t = [o.t;t];
            o.x_body = [o.x_body; y(:,1)];
            o.y_body = [o.y_body; y(:,3)];
            
            o.dx_body = [o.dx_body; y(:,2)];
            o.dy_body = [o.dy_body; y(:,4)];
            
            
            new_phi = o.phi_td*ones(size(t));
            new_phi(y(:,4) > 0) = o.phi(end); % only if y_dot > 0, update the phi.
            
            temp = max(find(y(:,4)>0));

            % for more smooth animation
            for i = 1:length(t) 
                if y(i,4) > 0
                    new_phi(i) = o.phi(end) + (o.phi_td - o.phi(end))*i/temp;  
                end
            end
            
            o.phi = [o.phi; new_phi];
            o.L = [o.L; o.L0*ones(size(t))]; % the leg will not change at flight stae.
            
            o.x_foot = [o.x_foot; y(:,1) + o.L0*sin(o.phi_td)];   
            
        case 1 % Stance
            o.t = [o.t;t];
            o.x_body = [o.x_body; y(:,1) + o.x_foot(end)];
            o.y_body = [o.y_body; y(:,3)];
            
            o.dx_body = [o.dx_body; y(:,2)];
            o.dy_body = [o.dy_body; y(:,4)];
             
            o.phi = [o.phi;atan2(-y(:,1),y(:,3))];
            o.L = [o.L;sqrt(y(:,1).^2 + y(:,3).^2)];
                        
            o.x_foot =  [o.x_foot; o.x_foot(end)*ones(size(t))];
    end 
end 

%function switchMode
function [o] = switchMode(o)
    
    switch o.dynamic_state
        case 0
            o.dynamic_state = 1; % Flight can only enter stance
        case 1
            o.dynamic_state = 0; % Stance can only enter flight
    end 

end 

%function flightTransitions
function [value,isterminal,direction] = flightTransitions(t,y,o)

    value = y(3) - o.L0*cos(o.phi_td);
    isterminal = 1;
    direction = -1;

end 

%function stanceTransitions
function [value,isterminal,direction] = stanceTransitions(t,y,o)

    value = sqrt(y(1)^2 + y(3)^2) - o.L0;
    isterminal = 1;
    direction = 1;

end 