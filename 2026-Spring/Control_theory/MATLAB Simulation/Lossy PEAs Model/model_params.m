function [m1, m2, Jm, Jw, Jmot, lo, b, g, k] = model_params

    m1 = 0.95;   % upper body mass
    m2 = 0.05;   % lower body mass
    Jm = 0.5;    % inertia of upper and lower body
    Jw = 0.5;     % inertia of wheel
    Jmot = 0.01;  % reflected inertia

    lo = 1; 

    b = 1;
    g = 9.81;
    k = 100;
    
end
