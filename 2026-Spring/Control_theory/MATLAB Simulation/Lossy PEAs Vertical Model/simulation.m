%% Poincare from the ground
close all; clear all; clc

global g_old_lambda   % store the last lambda
global g_motor_state  % 1: outputs torque; 0: no torque
global g_stimu_time   % store the torque start time
global g_stimu_time_  % store the torque end time
global g_stimu_time_flag % flag that prevents multiple store torque end time


g_old_lambda = 0;
g_motor_state = 0;
g_stimu_time = [];
g_stimu_time_ = [];
g_stimu_time_flag = 0;

% poincare map section
Z0 = [0.4];  %  [y];

%Converges after 1 iteration
numIters = 1;
Z0_periodic = zeros(1,numIters);

opts = optimoptions('fsolve', ...
    'Algorithm', 'levenberg-marquardt', ... 
    'FunctionTolerance', 1e-10, ...
    'StepTolerance', 1e-8, ...
    'FiniteDifferenceStepSize', 1e-6, ...
    'MaxIterations', 1000, ...
    'Display', 'iter-detailed'); 

for i = 1:numIters
    [Z0_periodic(:,i), ~, ~, ~, jacobian] = fsolve(@(Z)(Poincare_map_ground(Z) - Z), Z0, opts);
    Z0 = Z0_periodic(:,i)
end

jacobian_real = jacobian + eye(size(jacobian));
eigen = eig(jacobian_real)
lambda = abs(eigen)

%% solve ode45 (ground)
close all; clear all; clc

global g_old_lambda   % store the last lambda
global g_motor_state  % 1: outputs torque; 0: no torque
global g_stimu_time   % store the torque start time
global g_stimu_time_  % store the torque end time
global g_stimu_time_flag % flag that prevents multiple store torque end time


g_old_lambda = 0;
g_motor_state = 0;
g_stimu_time = [];
g_stimu_time_ = [];
g_stimu_time_flag = 0;


X0 = [0.3; 0.3; 0; 0];  %  [y; l; dy; dl];

dynamics_status = 0;  % 0: ground  1: flight

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

        X0 = [X(end,1); X(end,2); X(end,3); X(end,3)];
        dynamics_status = 0;
        disp('dynamics_status: flight'); 

    else
        
        tspan = t_start:0.01:t_stop;
        [t,X,~,~,ie] = ode45(@(t,X)sys_ground(t,X),tspan,X0,op_ground);

        X_data = [X_data; X];
        t_data = [t_data; t];
        
        X0 = [X(end,1); X(end,2); X(end,3); X(end,4)];
        dynamics_status = 1;
        disp('dynamics_status: ground');
        
        
    end

    t_start = t(end);
    
end

y = X_data(:, 1);
l = X_data(:, 2);
dy = X_data(:, 3);
dl = X_data(:, 4);

% %% Plot y and stimulation

figure('Position', [250, 250, 800, 400]);

hold on;
plot(t_data, y, 'r','LineWidth', 1);
plot(g_stimu_time(1), 1, 'b-*','LineWidth', 1);
plot(g_stimu_time_(1), 1, 'g-*','LineWidth', 1);

plot(g_stimu_time, 1, 'b-*','LineWidth', 1);
plot(g_stimu_time_, 1, 'g-*','LineWidth', 1);

% plot([0 t_data(end)], [y(1) y(1)], 'k-.','LineWidth', 1);
% plot([0 t_data(end)], [y(end) y(end)], 'k-.','LineWidth', 1);

for i = 1:length(g_stimu_time_)
    rectangle('Position', [g_stimu_time(i) -1 g_stimu_time_(i)-g_stimu_time(i) 10], ... 
              'FaceColor', 'blue', ...   
              'FaceAlpha', 0.1);  
end   

xlabel('Time [s]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('Distance [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
legend('$y$','Stimu. start','Stimu. stop','Interpreter','latex','fontsize',20,'location','ne')
ylim([0.6 1.4])
grid on;
saveas(gcf, 'y-stimulation.png');

figure('Position', [250, 250, 800, 400]);
hold on;
plot(dy, y, 'LineWidth', 1);
xlabel('$y$ [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$\dot y$ [$m/s$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'phase y-dy.png');


%% Poincare from the flight
close all; clear all; clc

global g_old_lambda   % store the last lambda
global g_motor_state  % 1: outputs torque; 0: no torque
global g_stimu_time   % store the torque start time
global g_stimu_time_  % store the torque end time
global g_stimu_time_flag % flag that prevents multiple store torque end time


g_old_lambda = 0;
g_motor_state = 0;
g_stimu_time = [];
g_stimu_time_ = [];
g_stimu_time_flag = 0;

% poincare map section
Z0 = [1.5];  %  [y];

%Converges after 1 iteration
numIters = 1;
Z0_periodic = zeros(1,numIters);

opts = optimoptions('fsolve', ...
    'Algorithm', 'levenberg-marquardt', ... 
    'FunctionTolerance', 1e-10, ...
    'StepTolerance', 1e-8, ...
    'FiniteDifferenceStepSize', 1e-6, ...
    'MaxIterations', 1000, ...
    'Display', 'iter-detailed'); 

for i = 1:numIters
    [Z0_periodic(:,i), ~, ~, ~, jacobian] = fsolve(@(Z)(Poincare_map_flight(Z) - Z), Z0, opts);
    Z0 = Z0_periodic(:,i)
end

jacobian_real = jacobian + eye(size(jacobian));
eigen = eig(jacobian_real)
lambda = abs(eigen)

%% solve ode45 (flight)
close all; clear all; clc

global g_old_lambda   % store the last lambda
global g_motor_state  % 1: outputs torque; 0: no torque
global g_stimu_time   % store the torque start time
global g_stimu_time_  % store the torque end time
global g_stimu_time_flag % flag that prevents multiple store torque end time


g_old_lambda = 0;
g_motor_state = 0;
g_stimu_time = [];
g_stimu_time_ = [];
g_stimu_time_flag = 0;

X0 = [1.2816; 1; 0; 0];  %  [y; l; dy; dl];

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

        X0 = [X(end,1); X(end,2); X(end,3); X(end,3)];
        dynamics_status = 0;
        disp('dynamics_status: flight'); 
        
        
    else
        
        tspan = t_start:0.01:t_stop;
        [t,X,~,~,ie] = ode45(@(t,X)sys_ground(t,X),tspan,X0,op_ground);

        X_data = [X_data; X];
        t_data = [t_data; t];
        
        X0 = [X(end,1); X(end,2); X(end,3); X(end,4)];
        dynamics_status = 1;
        disp('dynamics_status: ground');
        
    end

    t_start = t(end);

end

y = X_data(:, 1);
l = X_data(:, 2);
dy = X_data(:, 3);
dl = X_data(:, 4);

% %% Plot y and stimulation

figure('Position', [250, 250, 800, 400]);

hold on;
plot(t_data, y, 'r','LineWidth', 1);
plot(g_stimu_time(1), 1, 'b-*','LineWidth', 1);
plot(g_stimu_time_(1), 1, 'g-*','LineWidth', 1);

plot(g_stimu_time, 1, 'b-*','LineWidth', 1);
plot(g_stimu_time_, 1, 'g-*','LineWidth', 1);

% plot([0 t_data(end)], [y(1) y(1)], 'k-.','LineWidth', 1);
% plot([0 t_data(end)], [y(end) y(end)], 'k-.','LineWidth', 1);

for i = 1:length(g_stimu_time_)
    rectangle('Position', [g_stimu_time(i) -1 g_stimu_time_(i)-g_stimu_time(i) 10], ... 
              'FaceColor', 'blue', ...   
              'FaceAlpha', 0.1);  
end   

xlabel('Time [s]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('Distance [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
legend('$y$','Stimu. start','Stimu. stop','Interpreter','latex','fontsize',20,'location','ne')
ylim([0.5 1.6])
grid on;
saveas(gcf, 'y-stimulation.png');

figure('Position', [250, 250, 800, 400]);
hold on;
plot(dy, y, 'LineWidth', 1);
xlabel('$y$ [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$\dot y$ [$m/s$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'phase y-dy.png');

%% Plot l

figure('Position', [250, 250, 800, 400]);
hold on;
plot(t_data, l, 'r','LineWidth', 1);
plot(g_stimu_time(1), 1, 'b-*','LineWidth', 1);
plot(g_stimu_time_(1), 1, 'g-*','LineWidth', 1);

plot(g_stimu_time, 1, 'b-*','LineWidth', 1);
plot(g_stimu_time_, 1, 'g-*','LineWidth', 1);

for i = 1:length(g_stimu_time_)
    rectangle('Position', [g_stimu_time(i) -1 g_stimu_time_(i)-g_stimu_time(i) 2.5], ... 
              'FaceColor', 'blue', ...   
              'FaceAlpha', 0.1);  
end   
xlabel('Time [s]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('l [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
legend('$l$','Stimu. start','Stimu. stop','Interpreter','latex','fontsize',20,'location','se')
grid on;
ylim([0.6 1.1])
saveas(gcf, 'l-stimulation.png');


%% Plot lambda

% find stimulation time
uforce_values = zeros(size(t_data)); 
for k = 1:length(g_stimu_time) 
    mask = (t_data >= g_stimu_time(k)) & (t_data <= g_stimu_time_(k));
    uforce_values(mask) = 10;
end

% calculate lambda_n
lambda_n = zeros(size(t_data));
for i = 1:length(t_data)
    [~, lambda_n(i)] = dyn_sol_ground([y(i), l(i)], [dy(i), dl(i)], uforce_values(i));
end

figure('Position', [250, 250, 800, 400]);
hold on;
plot(t_data, lambda_n, 'r' ,'LineWidth', 1);
plot(g_stimu_time(1), 1, 'b-*','LineWidth', 1);
plot(g_stimu_time_(1), 1, 'g-*','LineWidth', 1);
plot(g_stimu_time, 1, 'b-*','LineWidth', 1);
plot(g_stimu_time_, 1, 'g-*','LineWidth', 1);

for i = 1:length(g_stimu_time_)
    rectangle('Position', [g_stimu_time(i) -15 g_stimu_time_(i)-g_stimu_time(i) 80], ... 
              'FaceColor', 'blue', ...   
              'FaceAlpha', 0.1);  
end  

xlabel('Time [s]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$\lambda_n$ [$N$]', 'Interpreter', 'latex', 'fontsize', 20);
legend('$\lambda_n$','Stimu. start','Stimu. stop','Interpreter','latex','fontsize',20,'location','ne')
grid on;
ylim([-10 50])
saveas(gcf, 'lambda_n-stimulation.png');

%% Phase plot

figure('Position', [250, 250, 800, 400]);
hold on;
plot(dy, y, 'LineWidth', 1);
xlabel('$y$ [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$\dot y$ [$m/s$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'phase y-dy.png');

figure('Position', [250, 250, 800, 400]);
hold on;
plot(dl, l, 'LineWidth', 1);
xlabel('$l$ [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$\dot l$ [$m/s$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'phase l-dl.png');


%% Animation

[~, ~, ~, lo, ~, ~, ~] = model_params;
foot = y - l; % foot position
x = linspace(0, 0, length(t_data))';  
wheel_radius = 0.35;

% ====== gif ======
save_gif_switch = true; % gif button
gif_filename = 'animation.gif';
% ===================

figure;
axis equal;
xlim([-1, 1]);
ylim([-0.1, 3]);
grid on;
xlabel('X[m]');
ylabel('Y[m]');
hold on;

% ground 
ground_patch = patch([-1,-1,1,1],[0,-0.5,-0.5,0],[0.8,0.8,0.8]);

% init the components
[wheel_rim, wheel_spokes, wheel_hub] = init_wheel(x(1), y(1), wheel_radius);
m1 = plot(0, y(1), 'o', 'MarkerSize', 20, 'MarkerFaceColor', [0.6,0.8,1]);
m2 = plot(0, foot(1), 'o', 'MarkerSize', 12, 'MarkerFaceColor', [0.6,0.8,1]);
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
   
    update_spring(spring_line, fixed_points, 0, y(i), y(i)-l(i));
    
    set(m1, 'XData', x(i), 'YData', y(i));
    set(m2, 'XData', x(i), 'YData', foot(i));
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