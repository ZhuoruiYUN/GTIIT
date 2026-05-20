function [q_dd, LAMBDA]=dyn_sol_ground(q,q_d,u_force)
    
    [M,B,G,W] = dynamics_mat(q,q_d);
    
    LAMBDA = (W*(M\W')) \ (W*(M\(B + G - u_force)));
    q_dd = M \ (u_force + W' * LAMBDA - B - G);
  
end