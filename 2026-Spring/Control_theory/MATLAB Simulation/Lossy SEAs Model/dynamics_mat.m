function [M,B,G,W]  = dynamics_mat(q,q_d)
    
    [m1, m2, m3, Im, Jm, Jw, lo, l1, l2, b, g, k] = model_params;
    
    x = q(1);
    y = q(2);
    th = q(3);
    l = q(4);
    phi = q(5);
    u = q(6);

    dx = q_d(1);
    dy = q_d(2);
    dth = q_d(3);
    dl = q_d(4);
    dphi = q_d(5);
    du = q_d(6);

    M = zeros(6,6);
    M(1,1) = m1 + m2 + m3;
    M(1,2) = 0;
    M(1,3) = -cos(th)*(l1*m2 + m3*l);
    M(1,4) = -m3*sin(th);
    M(1,5) = 0;
    M(1,6) = 0;
    M(2,1) = 0;
    M(2,2) = m1 + m2 + m3;
    M(2,3) = sin(th)*(l1*m2 + m3*l);
    M(2,4) = -m3*cos(th);
    M(2,5) = 0;
    M(2,6) = 0;
    M(3,1) = -cos(th)*(l1*m2 + m3*l);
    M(3,2) = sin(th)*(l1*m2 + m3*l);
    M(3,3) = m2*l1^2 + m3*l^2 + Im + Jw;
    M(3,4) = 0;
    M(3,5) = Jw;
    M(3,6) = 0;
    M(4,1) = -m3*sin(th);
    M(4,2) = -m3*cos(th);
    M(4,3) = 0;
    M(4,4) = m3;
    M(4,5) = 0;
    M(4,6) = 0;
    M(5,1) = 0;
    M(5,2) = 0;
    M(5,3) = Jw;
    M(5,4) = 0;
    M(5,5) = Jw;
    M(5,6) = 0;
    M(6,1) = 0;
    M(6,2) = 0;
    M(6,3) = 0;
    M(6,4) = 0;
    M(6,5) = 0;
    M(6,6) = Jm;
    
    
    B = zeros(6,1);
    B(1) = m3*sin(th)*l*dth^2 - 2*m3*cos(th)*dl*dth + l1*m2*sin(th)*dth^2;
    B(2) = m3*cos(th)*l*dth^2 + 2*m3*sin(th)*dl*dth + l1*m2*cos(th)*dth^2;
    B(3) = 2*m3*l*dl*dth ...
         - 2*m3*cos(th)*dl*dx + 2*m3*sin(th)*dl*dy ...
         + 2*l1*m2*cos(th)*dth*dy + 2*l1*m2*sin(th)*dth*dx ...
         + 2*m3*cos(th)*l*dth*dy + 2*m3*sin(th)*l*dth*dx;
    B(4) = b*dl - b*du + m3*l*dth^2 ...
         - 2*m3*cos(th)*dth*dx + 2*m3*sin(th)*dth*dy;
    B(5) = 0;
    B(6) = -b*(dl - du);

    G = zeros(6,1);
    G(1) = 0;
    G(2) = g*(m1 + m2 + m3);
    G(3) = g*sin(th)*(l1*m2 + m3*l);
    G(4) = -0.5*k*(2*l1 + 2*l2 + 2*lo - 2*l + 2*u) - g*m3*cos(th);
    G(5) = 0;
    G(6) = k*(l1 + l2 + lo - l + u);

    wt = [1 0 -l*cos(th) -sin(th) 0 0];
    wn = [0 1 sin(th)*l -cos(th) 0 0];
    W = [wt;wn];
    
end