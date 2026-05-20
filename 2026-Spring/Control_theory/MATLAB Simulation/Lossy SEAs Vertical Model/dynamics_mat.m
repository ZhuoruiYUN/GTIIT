function [M,B,G,wn,dwn]  = dynamics_mat(q,q_d)
    
    [m1, m2, m3, jmot, lo, l1, l2, b, g, k] = model_params;

    l_ = q(2);
    u_ = q(3);

    l_d = q_d(2);
    u_d = q_d(3);

    M = zeros(3,3);
    M(1,1) = m1+m2+m3;
    M(1,2) = -m3;
    M(2,1) = M(1,2);
    M(2,2) = m3;
    M(3,3) = jmot;
 
    B = zeros(3,1);
    B(2) = b*(l_d-u_d);
    B(3) = -b*(l_d-u_d);

    G = zeros(3,1);
    G(1) = (m1 + m2 + m3) * g;
    G(2) = -m3 * g - k * (lo + l1 + l2 + u_ - l_);
    G(3) = k * (lo + l1 + l2 + u_ - l_);

    wn = [1 -1 0];
    
end