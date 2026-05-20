function theta_td = raibertHopperCtrl(dx_body, L, des_vel)

    kv = 0.1; 
    Ts = 0.09; % 0.09 s is prediction stance duration.
    compen_impact = 1.9;

    % compen_impact is a factor that compensate the lossing of dx from impact
    x_foot = 1/2 * Ts * dx_body + kv * (dx_body - des_vel * compen_impact); 
    
    % Desired touchdown leg angle
    theta_td = asin(-x_foot / L);

end