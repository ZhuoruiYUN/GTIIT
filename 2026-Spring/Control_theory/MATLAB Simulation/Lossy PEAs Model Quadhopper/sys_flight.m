function [dXdt, F1, F2] = sys_flight(t,X)
    
    % calculate velocity of m1
    dx = X(5);
    theta = X(3);
    dtheta = X(7);
    l = X(4);
    dl = X(8);
   
    vx_m1 = (dx + dl*sin(theta) + dtheta*l*cos(theta));
    %
    
    global g_wheel_motor_flag;
    
    if g_wheel_motor_flag == 1
        theta_des = raibertHopperCtrl(vx_m1, l, 1);
        theta_d_des = 0;

        % detect if it reaches the destination
        angle_tolerance = 0.1;  
        if abs(theta - theta_des) < angle_tolerance && abs(dtheta - theta_d_des) < angle_tolerance
            F1 = 0;
            F2 = 0;
        else
            [F1, F2] = pid_controller(t, X, theta_des, theta_d_des);
        end
    else
        F1 = 0;
        F2 = 0;
    end
    

    q = X(1:4);
    q_d = X(5:8);

    q_dd = dyn_sol_flight(q, q_d, F1, F2);

    dXdt = [q_d; q_dd];
    
end