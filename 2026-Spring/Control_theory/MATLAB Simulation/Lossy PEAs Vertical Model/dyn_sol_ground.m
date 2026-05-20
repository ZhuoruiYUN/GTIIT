function [q_dd, LAMBDA]=dyn_sol_ground(q,q_d,u_force)

    [M,B,G,wn] = dynamics_mat(q,q_d);
    

    F = [0;
         u_force];

    LAMBDA = (wn*(M\wn')) \ (wn*(M\(B + G - F)));
    q_dd = M \ (F + wn' * LAMBDA  - B - G);
    
end