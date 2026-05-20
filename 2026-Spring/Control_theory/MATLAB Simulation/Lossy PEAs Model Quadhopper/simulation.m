%% solve ode45 (flight)
close all; clear all; clc

function [dXdt, F1_out, F2_out] = sys_flight_wrapper(t, X)
    [dXdt, F1_out, F2_out] = sys_flight(t, X);
    
    % 使用持久变量保存历史数据
    persistent F1_hist F2_hist t_hist
    
    if isempty(F1_hist)
        F1_hist = [];
        F2_hist = [];
        t_hist = [];
    end
    
    F1_hist = [F1_hist; F1_out];
    F2_hist = [F2_hist; F2_out];
    t_hist = [t_hist; t];
    
    % 返回最新的力值
    F1_out = F1_hist(end);
    F2_out = F2_hist(end);
end


global g_linear_motor_flag % 1: start linear motor; 0: stop linear motor
global g_wheel_motor_flag % 1: start wheel motor; 0: stop wheel motor
global g_old_lambda   % store the last lambda
global g_motor_state  % 1: outputs torque; 0: no torque
global g_stimu_time   % store the torque start time
global g_stimu_time_  % store the torque end time
global g_stimu_time_flag % flag that prevents multiple store torque end time

g_linear_motor_flag = 1; 
g_wheel_motor_flag = 0; 
g_old_lambda = 0;
g_motor_state = 0;
g_stimu_time = [];
g_stimu_time_ = [];
g_stimu_time_flag = 0;

[~, ~, ~, lo, ~, ~, ~, ~] = model_params;

X0 = [0; 1; 0; lo;    % [ x;  y;  theta;   l;
      0; 0; 0;  0];   %   dx; dy;  dtheta; dl];


global g_uforce
if g_linear_motor_flag == 1
    g_uforce = 40;
else
    g_uforce = 0;
end

dynamics_status = 1;  % 0: ground  1: flight

t_start = 0;
t_stop = 10;
op_ground = odeset('RelTol', 1e-6, 'AbsTol', 1e-6,'Events',@events_ground);    
op_flight = odeset('RelTol', 1e-6, 'AbsTol', 1e-6,'Events',@events_flight); 

X_data = [];
t_data = [];

% 在主循环前初始化
F1_data = [];
F2_data = [];

while abs(t_stop-t_start) > 1e-2

    if dynamics_status == 1
        tspan = t_start:0.01:t_stop;
        [t,X,~,~,ie] = ode45(@(t,X)sys_flight_wrapper(t,X),tspan,X0,op_flight);
       

        % 获取保存的力数据
        F1_current = zeros(size(t));
        F2_current = zeros(size(t));
        for i = 1:length(t)
            [~, F1_current(i), F2_current(i)] = sys_flight_wrapper(t(i), X(i,:)');
        end
        
        % 保存数据
        F1_data = [F1_data; F1_current];
        F2_data = [F2_data; F2_current];


        X_data = [X_data; X];
        t_data = [t_data; t];

        Xold = [X(end,1);  % x
                X(end,2);  % y
                X(end,3);  % theta 
                X(end,4);  % l 
                X(end,5);  % dx
                X(end,6);  % dy  
                X(end,7);  % dtheta
                X(end,8)];  % dl 
        
        dx = Xold(5);
        dl = Xold(8);
        theta = Xold(3);
        l = Xold(4);
        dtheta = Xold(7);

        bi = dx + dl*sin(theta) + dtheta*l*cos(theta); % before impact

        X0 = impact_law(Xold);
        
        dx = X0(5);
        dl = X0(8);
        theta = X0(3);
        l = X0(4);
        dtheta = X0(7);

        ai = dx + dl*sin(theta) + dtheta*l*cos(theta); % after impact

        dynamics_status = 0;
        disp('dynamics_status: flight'); 
            
    else
        
        tspan = t_start:0.01:t_stop;
        [t,X,~,~,ie] = ode45(@(t,X)sys_ground(t,X),tspan,X0,op_ground);

        % 地面阶段力为零
        F1_data = [F1_data; zeros(size(t))];
        F2_data = [F2_data; zeros(size(t))];


        X_data = [X_data; X];
        t_data = [t_data; t];        
        
        X0 = [X(end,1);  % x
              X(end,2);  % y
              X(end,3);  % theta 
              X(end,4);  % l 
              X(end,5);  % dx
              X(end,6);  % dy
              X(end,7);  % dtheta
              X(end,8)];   % dl

        dynamics_status = 1;
        disp('dynamics_status: ground');
       
    end

    t_start = t(end);

end

x = X_data(:, 1);
y = X_data(:, 2);
theta = X_data(:, 3);
l = X_data(:, 4);
dx = X_data(:, 5);
dy = X_data(:, 6);
dtheta = X_data(:, 7);
dl = X_data(:, 8);


%% Animation

addpath(genpath('f_animation'));
if ~exist('figs', 'dir'); mkdir('figs'); end

[~, ~, ~, lo, lc, ~, ~, ~] = model_params;

% ====== gif ======
save_gif_switch = false ; % gif button
gif_filename = 'figs/animation.gif';
% ===================

figure('Position', [300, 300, 1800, 220]);
xlim([-5, 20]);
ylim([-0.1, 3.8]);
grid on;
xlabel('X[m]');
ylabel('Y[m]');
hold on;



F1_text = text(-4, 3.5, 'F1: 0.0 N', 'FontSize', 12);
F2_text = text(-4, 3.2, 'F2: 0.0 N', 'FontSize', 12);


max_F1 = max(abs(F1_data));
max_F2 = max(abs(F2_data));

% ground 
ground_patch = patch([-20,-20,20,20],[0,-0.5,-0.5,0],[0.8,0.8,0.8]);

x_m1_init = x(1) + l(1) * sin(theta(1));
y_m1_init = y(1) + l(1) * cos(theta(1));

% calculate the flight axis
x_offset_init = (lc / 2) * cos(theta(1));
y_offset_init = (lc / 2) * sin(theta(1));
x_end1_init = x_m1_init - x_offset_init;
x_end2_init = x_m1_init + x_offset_init;
y_end1_init = y_m1_init + y_offset_init;
y_end2_init = y_m1_init - y_offset_init;


% init the components
m1 = plot(x_m1_init, y_m1_init, 'o', 'MarkerSize', 20, 'MarkerFaceColor', [0.6,0.8,1]);
rod_line = plot([x_end1_init, x_end2_init], [y_end1_init, y_end2_init], 'LineWidth', 5, 'Color', [0.6 0.2 0.3]); 
propeller1 = plot(x_end1_init, y_end1_init, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.6,0.8,1]);
propeller2 = plot(x_end2_init, y_end2_init, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.6,0.8,1]);
m2 = plot(x(1), y(1), 'o', 'MarkerSize', 12, 'MarkerFaceColor', [0.6,0.8,1]);
[spring_line, fixed_points] = init_spring(x_m1_init, y_m1_init, x(1), y(1));


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
   
    % current F
    current_F1 = F1_data(i);
    current_F2 = F2_data(i);
    
   
    set(F1_text, 'String', sprintf('F1: %.1f N', current_F1));
    set(F2_text, 'String', sprintf('F2: %.1f N', current_F2));
    
    % color gradient
    if max_F1 > 0
        F1_color = [1, max(0, 0.7 - 0.7*(abs(current_F1)/max_F1)), max(0, 0.7 - 0.7*(abs(current_F1)/max_F1))];
    else
        F1_color = [1, 0.7, 0.7]; 
    end
    
    if max_F2 > 0
        F2_color = [1, max(0, 0.7 - 0.7*(abs(current_F2)/max_F2)), max(0, 0.7 - 0.7*(abs(current_F2)/max_F2))];
    else
        F2_color = [1, 0.7, 0.7]; % 
    end

    % update color
    set(propeller1, 'MarkerFaceColor', F1_color); %
    set(propeller2, 'MarkerFaceColor', F2_color); %

    x_m1_new = x(i) + l(i) * sin(theta(i));
    y_m1_new = y(i) + l(i) * cos(theta(i));

    x_offset_new = (lc / 2) * cos(theta(i));
    y_offset_new = (lc / 2) * sin(theta(i));

    x_end1_new = x_m1_new - x_offset_new;
    y_end1_new = y_m1_new + y_offset_new;
    x_end2_new = x_m1_new + x_offset_new;
    y_end2_new = y_m1_new - y_offset_new;

    update_spring(spring_line, fixed_points, x_m1_new, y_m1_new, x(i), y(i));
    set(m1, 'XData', x_m1_new, 'YData', y_m1_new);
    set(m2, 'XData', x(i), 'YData', y(i));
    set(rod_line, 'XData', [x_end1_new, x_end2_new], 'YData', [y_end1_new, y_end2_new]);
    set(propeller1, 'XData', x_end1_new, 'YData', y_end1_new);
    set(propeller2, 'XData', x_end2_new, 'YData', y_end2_new);
    drawnow;

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


%% Phase plot

figure('Position', [250, 250, 800, 400]);
hold on;
plot(y, dy, 'LineWidth', 1);
xlabel('$y$ [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$\dot y$ [$m/s$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'figs/phase y-dy.png');

figure('Position', [250, 250, 800, 400]);
hold on;
plot(theta*180/pi, dtheta*180/pi, 'LineWidth', 1);
xlabel('$\theta$ [$^\circ$]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$\dot \theta$ [$^\circ/s$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'figs/phase th-dth.png');




