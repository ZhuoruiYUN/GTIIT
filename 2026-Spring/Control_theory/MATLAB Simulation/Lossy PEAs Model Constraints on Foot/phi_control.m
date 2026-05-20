function [phi_d, phi_dd] = phi_control(t, X, theta_des, theta_d_des)

    Kp = 500;   
    Kd = 40;   


    theta = X(3);         
    dtheta = X(8);        

    dphi = X(10);        

   
    theta_error = theta - theta_des;
    dtheta_error = dtheta - theta_d_des;


    phi_dd = Kp * theta_error + Kd * dtheta_error;


    phi_d = dphi + phi_dd * 0.01; % tims step 0.01s
end
