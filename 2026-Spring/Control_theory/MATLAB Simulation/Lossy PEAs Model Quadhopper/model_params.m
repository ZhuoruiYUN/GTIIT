function [m1, m2, Jm, lo, lc, b, g, k] = model_params

    m1 = 0.95;   % upper body mass
    m2 = 0.05;   % lower body mass
    Jm = 1;    % inertia of upper and lower body
    lo = 1; 
    lc = 4;
    b = 5;
    g = 9.81;
    k = 1000;
    
end
