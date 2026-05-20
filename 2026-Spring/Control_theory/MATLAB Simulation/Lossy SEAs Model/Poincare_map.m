function Znew = Poincare_map(Zold)


    X0 = [0;   Zold(1);  0;  Zold(2);   0;   0;   % [x; y; th; l ; phi; u];
      0;   0;  0;    0;   0;   0;];  

    
    % [~, ~, ~, ~] = u_input(0, X0, true); 
    [~, ~, ~] = phi_input(0, X0, true); 
    
    dynamics_status = 1;  % 0: ground  1: flight
    
    t_start = 0;
    t_stop = 5;
    op_ground = odeset('RelTol', 1e-8, 'AbsTol', 1e-8,'Events',@events_ground);    
    op_flight = odeset('RelTol', 1e-8, 'AbsTol', 1e-8,'Events',@events_flight); 
    
    
    while abs(t_stop-t_start) > 1e-2
    
        if dynamics_status == 1
            tspan = t_start:0.01:t_stop;
            [t,X,~,~,~] = ode45(@(t,X)sys_flight(t,X), tspan, X0, op_flight);
            t_start = t(end);
            X0 = [0; X(end,2); 0; X(end,4); X(end,5); X(end,6);
                  0; X(end,8); 0; X(end,8); X(end,11); X(end,12)];
            dynamics_status = 0;
            disp('dynamics_status: flight');   
            
    
        elseif dynamics_status == 0
    
            tspan = t_start:0.01:t_stop;
            [t,X,~,~,~] = ode45(@(t,X)sys_ground(t,X),tspan,X0,op_ground);
            t_start = t(end);
            X0 = [X(end,1); X(end,2); X(end,3); X(end,4); X(end,5); X(end,6);
                  X(end,7); X(end,8); X(end,9); X(end,10); X(end,11); X(end,12)];
            dynamics_status = 1;
            disp('dynamics_status: ground');
    
        end
    end

    Znew = [X0(2); X0(4)]
    
end