function theta_td = raibertHopperCtrl(dx_body, L, des_vel)

    kv = 0.1; 
    Ts = 0.08; % 0.08 s is prediction stance duration.
    compen_impact = 2;

    % compen_impact is a factor that compensate the lossing of dx from impact
    x_foot = 1/2 * Ts * dx_body + kv * (dx_body - des_vel * compen_impact); 
    
    % Desired touchdown leg angle
    theta_td = asin(-x_foot / L);

end