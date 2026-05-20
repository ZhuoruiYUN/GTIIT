function Znew = Poincare_map(Zold)


    X0 = [Zold(1); Zold(2); 0; 0; 0; 0];

    [~, ~, ~, ~] = motor_input(0, X0, true)

    dynamics_status = 1;  % 0: ground  1: flight

    t_start = 0;
    t_stop = 10;
    
    op_ground = odeset('RelTol', 1e-8, 'AbsTol', 1e-8,'Events',@events_ground);    
    op_flight = odeset('RelTol', 1e-8, 'AbsTol', 1e-8,'Events',@events_flight); 
   
    
    while abs(t_stop-t_start) > 1e-2
    
        if dynamics_status == 1

            tspan = t_start:0.01:t_stop;
            [t,X,~,~,~] = ode45(@(t,X)sys_flight(t,X),tspan,X0,op_flight);
            t_start = t(end);
            X0 = [X(end,1); X(end,2); 0; X(end,4); X(end,4); 0];
            dynamics_status = 0;
            
            
        elseif dynamics_status == 0
            
            tspan = t_start:0.01:t_stop;
            [t,X,~,~,~] = ode45(@(t,X)sys_ground(t,X),tspan,X0,op_ground);
            t_start = t(end);
            X0 = [X(end,1); X(end,2); X(end,3); X(end,4); X(end,5); X(end,6)];
            dynamics_status = 1;
            
        end
    end

    Znew = [X0(1); X0(2)];
    
end