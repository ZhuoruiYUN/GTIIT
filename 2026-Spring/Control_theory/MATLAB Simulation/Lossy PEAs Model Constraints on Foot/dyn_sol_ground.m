function [q_dd, LAMBDA]=dyn_sol_ground(q,q_d,u_force)

    [M,B,G,W] = dynamics_mat(q,q_d);
    
    
    F = [0;   % x
         0;   % y
         0;   % th
         u_force;  % l
         0];  % phi
    
    LAMBDA = (W*(M\W')) \ (W*(M\(B + G - F)));
    q_dd = M \ (F + W' * LAMBDA  - B - G);
    
end