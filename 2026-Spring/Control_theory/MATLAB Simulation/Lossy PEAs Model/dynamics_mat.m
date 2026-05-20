function [M, B, G, W]  = dynamics_mat(q,dq)
    
    [m1, m2, Jm, Jw, Jmot, lo, b, g, k] = model_params;

    theta = q(3);
    l = q(4);

    dx = dq(1);
    dy = dq(2);
    dtheta = dq(3);
    dl = dq(4);
    
    M = [m1 + m2 0 l*m2*cos(theta) m2*sin(theta) 0; 0 m1 + m2 l*m2*sin(theta) -m2*cos(theta) 0; l*m2*cos(theta) l*m2*sin(theta) Jm + Jw + l^2*m2 0 Jw; m2*sin(theta) -m2*cos(theta) 0 Jmot + m2 0; 0 0 Jw 0 Jw];
    B = [dtheta*m2*(2*dl*cos(theta) - dtheta*l*sin(theta)); dtheta*m2*(2*dl*sin(theta) + dtheta*l*cos(theta)); 2*m2*(dl*dx*cos(theta) + dl*dy*sin(theta) + dl*dtheta*l + dtheta*dy*l*cos(theta) - dtheta*dx*l*sin(theta)); b*dl + dtheta^2*l*m2 + 2*dtheta*dx*m2*cos(theta) + 2*dtheta*dy*m2*sin(theta); 0];
    G = [0; g*(m1 + m2); g*l*m2*sin(theta); (k*(2*l - 2*lo))/2 - g*m2*cos(theta); 0];


    wt = [1 0 -l*cos(theta) -sin(theta) 0]; % x-l*sin(theta)>=0
    wn = [0 1 l*sin(theta) -cos(theta) 0]; % y-l*cos(theta)>=0
    
    W = [wt; wn];
    
end