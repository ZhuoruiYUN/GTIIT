function [m1, m2, jmot, lo, b, g, k] = model_params

    m1 = 0.95;   % upper body mass
    m2 = 0.05;   % motor mass
    jmot = 0.01;  % reflected inertia

    lo = 1; 

    b = 1;
    g = 9.81;
    k = 100;
    
end
