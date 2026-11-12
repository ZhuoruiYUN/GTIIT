clc;
clear;
close all;

% Tank parameters
A = 2.5;
R = 4;

% Proportional controller
Kp = 5;

% Open-loop transfer function
s = tf('s');
G = R / (A*R*s + 1);

% Closed-loop transfer function
T = feedback(Kp*G, 1);

% Display transfer functions
disp('Open-loop Transfer Function:');
G

disp('Closed-loop Transfer Function:');
T

% Step response for 3 m reference
figure;
step(3*T, 5);
grid on;

title('Closed-loop Water Level Response');
xlabel('Time (s)');
ylabel('Water Level (m)');

% Steady-state value
h_ss = 3 * dcgain(T);

fprintf('Steady-state water level = %.4f m\n', h_ss);
fprintf('Steady-state error = %.4f m\n', 3-h_ss);