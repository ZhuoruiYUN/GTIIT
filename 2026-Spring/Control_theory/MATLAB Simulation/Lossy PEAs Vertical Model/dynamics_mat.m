function [M,B,G,wn]  = dynamics_mat(q,q_d)
    
    [m1, m2, jmot, lo, b, g, k] = model_params;

    l_ = q(2);
    l_d = q_d(2);

    M = zeros(2,2);
    M(1,1) = m1+m2;
    M(1,2) = -m2;
    M(2,1) = M(1,2);
    M(2,2) = jmot+m2;
 
    B = zeros(2,1);
    B(1) = 0;
    B(2) = b*l_d;

    G = zeros(2,1);
    G(1) = (m1+m2)*g;
    G(2) = -m2*g-k*(lo-l_);
    
    wn = [1 -1];
    
end