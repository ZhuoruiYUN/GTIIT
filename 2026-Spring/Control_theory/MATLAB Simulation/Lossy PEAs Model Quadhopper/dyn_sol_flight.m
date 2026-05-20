function q_dd= dyn_sol_flight(q,q_d,F1,F2)

    [~, ~, ~, ~, lc, ~, ~, ~] = model_params;
    
    theta = q(3);

    [M,B,G,~] = dynamics_mat(q,q_d);    

    F = [(F1+F2)*sin(theta);   % x
         (F1+F2)*cos(theta);   % y
         (F1-F2)*lc;   % th
         0];  % l
    
    q_dd = M \ (F - B - G);
    
end