%% Poincare
close all; clear all; clc

Z0 = [0.5714; 0.2911];  %  [y; l;];

%Converges after 1 iteration
numIters = 1;
Z0_periodic = zeros(2,numIters);

opts = optimoptions('fsolve', ...
    'Display',        'iter', ...     
    'Algorithm',      'trust-region-dogleg', ...
    'FunctionTolerance', 1e-10, ...
    'StepTolerance',     1e-10, ...
    'MaxIterations',     1000, ...
    'MaxFunctionEvaluations', 5000);

for i = 1:numIters
    [Z0_periodic(:,i), ~, ~, ~, jacobian] = fsolve(@(Z)(Poincare_map(Z) - Z), Z0, opts);
    Z0 = Z0_periodic(:,i)
end

jacobian_real = jacobian + eye(size(jacobian));
eigen = eig(jacobian_real)
lambda = abs(eigen)

%% Discrete Y

Z_star = [0.5714; 0.2911].'; 

num_steps = 15; 
Psi_k = zeros(1, num_steps);
Psi_k(1) = Z_star(1);

Z_current = Z_star;
for k = 2:num_steps
    Z_next = Poincare_map(Z_current);
    Psi_k(k) = Z_next(1);
    Z_current = Z_next;  
end

figure;
stem(1:num_steps, Psi_k, 'filled', 'b'); 
hold on;
plot([1 num_steps], [Z0(1) Z0(1)], 'k--', 'LineWidth', 1.5);  
title('Discrete-time series of y at impact','fontsize',20,'Interpreter','latex')
xlabel('Impact number k', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$Y_k$ [m]', 'Interpreter', 'latex', 'fontsize', 20);
ylim([-0.05, 0.7]);
legend("$Y_k$", "$y^*$", 'Interpreter','latex','fontsize',20,'location','ne')


%% solve ode45

X0 = [0.5714; 0.2911; 0; 0; 0; 0;];  %  [y; l; u; dy; dl; du;];
[~, ~, ~, ~] = motor_input(0, X0, true); 

dynamics_status = 1;  % 0: ground  1: flight

t_start = 0;
t_stop = 10;
op_ground = odeset('RelTol', 1e-8, 'AbsTol', 1e-8,'Events',@events_ground);    
op_flight = odeset('RelTol', 1e-8, 'AbsTol', 1e-8,'Events',@events_flight); 
X_data = [];
t_data = [];


while abs(t_stop-t_start) > 1e-2

    if dynamics_status == 1
        tspan = t_start:0.01:t_stop;
        [t,X,~,~,ie] = ode45(@(t,X)sys_flight(t,X),tspan,X0,op_flight);
        
        X_data = [X_data; X];
        t_data = [t_data; t];
        t_start = t(end);
        X0 = [X(end,1); X(end,2); 0; X(end,4); X(end,4); 0];
        dynamics_status = 0;
        disp('dynamics_status: flight');        
        

    elseif dynamics_status == 0
        
        tspan = t_start:0.01:t_stop;
        [t,X,~,~,ie] = ode45(@(t,X)sys_ground(t,X),tspan,X0,op_ground);

        X_data = [X_data; X];
        t_data = [t_data; t];
        t_start = t(end);
        X0 = [X(end,1); X(end,2); X(end,3); X(end,4); X(end,5); X(end,6)];
        dynamics_status = 1;
        disp('dynamics_status: ground');
        
    end
end

y = X_data(:, 1);
l = X_data(:, 2);
u = X_data(:, 3);
dy = X_data(:, 4);
dl = X_data(:, 5);
du = X_data(:, 6);

lambda_n = [];
for i = 1:length(t_data)
    [~,LAMBDA_N] = dyn_sol_ground([y(i) l(i) 0],[dy(i) dl(i) 0],0);
    lambda_n = [lambda_n LAMBDA_N];
end

%%
figure('Position', [250, 250, 800, 400]);
hold on;
plot(t_data, y-l, 'r-o','LineWidth', 2);
xlabel('Time [s]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('Distance [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
legend('$y$','$Stimulation$','Interpreter','latex','fontsize',20,'location','ne')
ylim([-0.1 0.3])
grid on;
saveas(gcf, 'y.png');

%% Plot y and stimulation

figure('Position', [250, 250, 800, 400]);
hold on;
plot(t_data, y, 'r-*','LineWidth', 2);
plot(t_data, u, 'b-','LineWidth', 4);
xlabel('Time [s]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('Distance [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
legend('$y$','$Stimulation$','Interpreter','latex','fontsize',20,'location','ne')
ylim([-0.1 0.7])
grid on;
saveas(gcf, 'y.png');

%% Plot l

figure('Position', [250, 250, 800, 400]);
hold on;
plot(t_data, l, 'b','LineWidth', 2);
xlabel('Time [s]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('l [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'l.png');


%% Phase plot

figure('Position', [250, 250, 800, 400]);
hold on;
plot(dy, y, 'LineWidth', 2);
xlabel('$y$ [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$\dot y$ [$m/s$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'phase y-dy.png');

figure('Position', [250, 250, 800, 400]);
hold on;
plot(dl, l, 'LineWidth', 2);
xlabel('$l$ [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$\dot l$ [$m/s$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'phase l-dl.png');

figure('Position', [250, 250, 800, 400]);
hold on;
plot(du, u, 'LineWidth', 2);
xlabel('$u$ [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$\dot u$ [$m/s$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'phase u-du.png');


%% Plot lambda

figure('Position', [250, 250, 800, 400]);
hold on;
plot(t_data, lambda_n, 'LineWidth', 2);
xlabel('Time [s]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$\lambda_n$ [$N$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'lambda_n.png');


%% Animation
[~, ~, ~, ~, lo, l1, l2, ~, ~, ~] = model_params;
foot = y - l; % m3 position
x = linspace(0, 0, length(t_data))';  
wheel_radius = 0.1;

% ====== gif ======
save_gif_switch = false; % gif button
gif_filename = 'simulation.gif';
% ===================

figure;
axis equal;
xlim([-0.5, 0.5]);
ylim([-0.1, 0.8]);
grid on;
xlabel('X[m]');
ylabel('Y[m]');
hold on;

% ground 
ground_patch = patch([-0.5,-0.5,0.5,0.5],[0,-0.5,-0.5,0],[0.8,0.8,0.8]);

% init the components

[wheel_rim, wheel_spokes, wheel_hub] = init_wheel(x(1), y(1), wheel_radius);
pole2 = plot([0,0], [y(1)-l1,y(1)-l1-u(1)], '-', 'LineWidth', 10, 'Color', [1,0,0]);
pole1 = plot([0,0], [y(1),y(1)-l1], '-', 'LineWidth', 5, 'Color', [0,0,0]);
pole3 = plot([0,0], [y(1)-l1-u(1)-(l(1)-l1-u(1)-l2),y(1)-l1-u(1)-(l(1)-l1-u(1)-l2)-l2], '-', 'LineWidth', 5, 'Color', [0,0,0]);
m1 = plot(0, y(1), 'o', 'MarkerSize', 20, 'MarkerFaceColor', [0.6,0.8,1]);
m2 = plot(0, y(1)-l1, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.6,0.8,1]);
m3 = plot(0, foot(1), 'o', 'MarkerSize', 12, 'MarkerFaceColor', [0.6,0.8,1]);
[spring_line, fixed_points] = init_spring(0.5, 0, -0.2);


% ====== gif ======
if save_gif_switch
    if exist(gif_filename, 'file')
        delete(gif_filename);
    end
    frame_count = 0;
end
% ===================

for i = 1:length(t_data)
    % update
   
    set(pole1, 'XData', [0,0], 'YData', [y(i),y(i)-l1]);
    set(pole2, 'XData', [0,0], 'YData', [y(i)-l1,y(i)-l1-u(i)]);
    set(pole3, 'XData', [0,0], 'YData', [y(i)-l1-u(i)-(l(i)-l1-u(i)-l2),y(i)-l1-u(i)-(l(i)-l1-u(i)-l2)-l2]);
    update_spring(spring_line, fixed_points, 0, y(i)-l1-u(i), y(i)-l1-u(i)-(l(i)-l1-u(i)-l2));
    
    set(m1, 'XData', x(i), 'YData', y(i));
    set(m2, 'XData', x(i), 'YData', y(i)-l1);
    set(m3, 'XData', x(i), 'YData', foot(i));
    update_wheel(wheel_rim, wheel_spokes, wheel_hub, x(i), y(i), wheel_radius, pi/3);
    drawnow;
    pause(0.02);
    
    % ====== gif ======
    if save_gif_switch
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        
        if frame_count == 0
            imwrite(imind, cm, gif_filename, 'gif', ...
                   'Loopcount', inf, ...
                   'DelayTime', 0.02);
        else
            imwrite(imind, cm, gif_filename, 'gif', ...
                   'WriteMode', 'append', ...
                   'DelayTime', 0.02);
        end
        
        frame_count = frame_count + 1;
    end
    % ===================
end