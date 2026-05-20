function [F1, F2] = pid_controller(t, X, theta_des, theta_d_des)
    
    Kp_theta = 5;     
    Kd_theta = 2;      
    
    
    theta = X(3);       
    dtheta = X(7);   
    
    
    theta_error = theta_des - theta;
    dtheta_error = theta_d_des - dtheta;
    
    
    F_diff = Kp_theta * theta_error + Kd_theta * dtheta_error;
    
    F_total = 2;

    if F_diff >= F_total
       F_diff = F_total;
    elseif F_diff <= -F_total
       F_diff = -F_total;
    end
    

    F1 = (F_total + F_diff) / 2;
    F2 = (F_total - F_diff) / 2;

    
    
end