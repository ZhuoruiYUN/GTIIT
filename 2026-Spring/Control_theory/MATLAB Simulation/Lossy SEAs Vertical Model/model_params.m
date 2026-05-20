function [m1, m2, m3, jmot, lo, l1, l2, b, g, k] = model_params

    m1 = 2;   % upper body mass
    m2 = 0.5;   % motor mass
    m3 = 0.2;   % lower body mass
    jmot = 0.5;  % reflected inertia

    lo = 0.2; 
    l1 = 0.05;
    l2 = 0.05;

    b = 5;
    g = 9.81;
    k = 800;
    
end
