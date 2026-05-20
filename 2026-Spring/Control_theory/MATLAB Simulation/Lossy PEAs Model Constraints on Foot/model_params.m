function [m1, m2, Jm, Jw, Jmot, lo, b, g, k] = model_params

    m1 = 0.95;   % upper body mass
    m2 = 0.05;   % lower body mass
    Jm = 1;    % inertia of upper and lower body
    Jw = 2;     % inertia of wheel
    Jmot = 0.01;  % reflected inertia

    lo = 1; 

    b = 5;
    g = 9.81;
    k = 1000;
    
end
