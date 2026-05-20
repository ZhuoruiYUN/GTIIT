function [M, B, G, W]  = dynamics_mat(q,dq)
    
    [m1, m2, Jm, Jw, Jmot, lo, b, g, k] = model_params;

    theta = q(3);
    l = q(4);

    dx = dq(1);
    dy = dq(2);
    dtheta = dq(3);
    dl = dq(4);
    
    M = [m1 + m2 0 l*m1*cos(theta) m1*sin(theta) 0; 0 m1 + m2 -l*m1*sin(theta) m1*cos(theta) 0; l*m1*cos(theta) -l*m1*sin(theta) Jm + Jw + l^2*m1 0 Jw; m1*sin(theta) m1*cos(theta) 0 Jmot + m1 0; 0 0 Jw 0 Jw];
    B = [dtheta*m1*(2*dl*cos(theta) - dtheta*l*sin(theta)); -dtheta*m1*(2*dl*sin(theta) + dtheta*l*cos(theta)); -2*m1*(dl*dy*sin(theta) - dl*dx*cos(theta) - dl*dtheta*l + dtheta*dy*l*cos(theta) + dtheta*dx*l*sin(theta)); b*dl + dtheta^2*l*m1 + 2*dtheta*dx*m1*cos(theta) - 2*dtheta*dy*m1*sin(theta); 0];
    G = [0; g*(m1 + m2); -g*l*m1*sin(theta); (k*(2*l - 2*lo))/2 + g*m1*cos(theta); 0];

    % foot constraints
    wt = [1 0 0 0 0]; 
    wn = [0 1 0 0 0]; 
    
    W = [wt; wn];
    
end