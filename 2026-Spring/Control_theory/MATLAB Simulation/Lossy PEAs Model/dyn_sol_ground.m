function [q_dd, LAMBDA]=dyn_sol_ground(q,q_d,u_force)

    [M,B,G,W] = dynamics_mat(q,q_d);

    theta = q(3);

    F = [-sin(theta) * u_force;
         cos(theta) * u_force;
         0;
         0;
         0]; 
    
    LAMBDA = (W*(M\W')) \ (W*(M\(B + G - F)));
    q_dd = M \ (F + W' * LAMBDA  - B - G);
    
end