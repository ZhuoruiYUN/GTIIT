function [m1, m2, m3, Im, Jm, Jw, lo, l1, l2, b, g, k] = model_params

    m1 = 1;   % upper body mass
    m2 = 0.1;   % motor mass
    m3 = 0.0001;   % lower body mass
    Im = 0.5;   % total body inertia


    Jm = 0.5;  % reflected inertia
    Jw = 0.5;  % wheel inertia

    lo = 0.2; 
    l1 = 0.05;
    l2 = 0.05;

    b = 0;
    g = 9.81;
    k = 300;
    
end
