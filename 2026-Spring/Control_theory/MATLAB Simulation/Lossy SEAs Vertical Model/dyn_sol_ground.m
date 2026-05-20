function [q_dd, LAMBDA]=dyn_sol_ground(q,q_d,u_force)

    [M,B,G,wn] = dynamics_mat(q,q_d);
    
    LAMBDA = (wn*(M\wn')) \ (wn*(M\(B + G - u_force)));
    q_dd = M \ (u_force + wn' * LAMBDA  - B - G);
  
end