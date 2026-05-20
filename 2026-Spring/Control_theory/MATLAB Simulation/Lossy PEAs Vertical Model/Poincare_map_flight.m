function Znew = Poincare_map_flight(Zold)

    X0 = [Zold(1); 1; 0; 0]; %[y; l; dy; dl];

    dynamics_status = 1;  % 0: ground  1: flight

    t_start = 0;
    t_stop = 10;

    op_ground = odeset('RelTol', 1e-8, 'AbsTol', 1e-8,'Events',@events_ground);    
    op_flight = odeset('RelTol', 1e-8, 'AbsTol', 1e-8,'Events',@events_flight_PM); 

    count = 0;
    while abs(t_stop-t_start) > 1e-2 

        if dynamics_status == 1

            tspan = t_start:0.01:t_stop;
            [t,X,~,~,~] = ode45(@(t,X)sys_flight(t,X),tspan,X0,op_flight);
            X0 = [X(end,1); X(end,2); X(end,3); X(end,3)];
            dynamics_status = 0;
            
            count = count + 1;

        else

            tspan = t_start:0.01:t_stop;
            [t,X,~,~,~] = ode45(@(t,X)sys_ground(t,X),tspan,X0,op_ground);
            X0 = [X(end,1); X(end,2); X(end,3); X(end,4)];
            dynamics_status = 1;
            
        end

        t_start = t(end);
        

        if count == 2
            break;
        end

    end

    Znew = [X0(1)]; 

end