%% solve ode45 (flight)
close all; clear all; clc

global g_linear_motor_flag % 1: start linear motor; 0: stop linear motor
global g_wheel_motor_flag % 1: start wheel motor; 0: stop wheel motor
global g_old_lambda   % store the last lambda
global g_motor_state  % 1: outputs torque; 0: no torque
global g_stimu_time   % store the torque start time
global g_stimu_time_  % store the torque end time
global g_stimu_time_flag % flag that prevents multiple store torque end time

g_linear_motor_flag = 1; 
g_wheel_motor_flag = 1; 
g_old_lambda = 0;
g_motor_state = 0;
g_stimu_time = [];
g_stimu_time_ = [];
g_stimu_time_flag = 0;

[~, ~, ~, ~, ~, lo, ~, ~, ~]  = model_params;

X0 = [0; 1; 0; lo; 0;    % [ x;  y;  theta;   l;  phi; 
      0; 0; 0;  0; 0];   %   dx; dy;  dtheta; dl; dphi;];


global g_uforce
if g_linear_motor_flag == 1
    g_uforce = 50;
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

while abs(t_stop-t_start) > 1e-2

    if dynamics_status == 1
        tspan = t_start:0.01:t_stop;
        [t,X,~,~,ie] = ode45(@(t,X)sys_flight(t,X),tspan,X0,op_flight);
        
        X_data = [X_data; X];
        t_data = [t_data; t];

        Xold = [X(end,1);  % x
                X(end,2);  % y
                X(end,3);  % theta 
                X(end,4);  % l 
                X(end,5);  % phi
                X(end,6);  % dx
                X(end,7);  % dy  
                X(end,8);  % dtheta
                X(end,9);  % dl 
                X(end,10)];  % dphi
        
        dx = Xold(6);
        dl = Xold(9);
        theta = Xold(3);
        l = Xold(4);
        dtheta = Xold(8);

        bi = dx + dl*sin(theta) + dtheta*l*cos(theta); % before impact

        X0 = impact_law(Xold);
        
        dx = X0(6);
        dl = X0(9);
        theta = X0(3);
        l = X0(4);
        dtheta = X0(8);

        ai = dx + dl*sin(theta) + dtheta*l*cos(theta); % after impact

        dynamics_status = 0;
        disp('dynamics_status: flight'); 
            
    else
        
        tspan = t_start:0.01:t_stop;
        [t,X,~,~,ie] = ode45(@(t,X)sys_ground(t,X),tspan,X0,op_ground);


        X_data = [X_data; X];
        t_data = [t_data; t];        
        
        X0 = [X(end,1);  % x
              X(end,2);  % y
              X(end,3);  % theta 
              X(end,4);  % l 
              X(end,5);  % phi
              X(end,6);  % dx
              X(end,7);  % dy
              X(end,8);  % dtheta
              X(end,9);  % dl
              X(end,10)];   % dphi

        dynamics_status = 1;
        disp('dynamics_status: ground');
           
       
    end

    t_start = t(end);

end

x = X_data(:, 1);
y = X_data(:, 2);
theta = X_data(:, 3);
l = X_data(:, 4);
phi = X_data(:, 5);
dx = X_data(:, 6);
dy = X_data(:, 7);
dtheta = X_data(:, 8);
dl = X_data(:, 9);
dphi = X_data(:, 10);

%%
x_m1 = [];
y_m1 = [];

vx_m1 = [];
vy_m1 = [];

for i = 1:length(x)
    x_m1 = [x_m1 x(i) + l(i) * sin(theta(i))];
    y_m1 = [y_m1 y(i) + l(i) * cos(theta(i))];
    vx_m1 = [vx_m1 dx(i) + dl(i)*sin(theta(i)) + dtheta(i)*l(i)*cos(theta(i))];
    vy_m1 = [vy_m1 dy(i) + dl(i)*cos(theta(i)) - dtheta(i)*l(i)*sin(theta(i))];
end

figure
plot(t_data,vx_m1);
xlabel('Time [s]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('Velocity [$m/s$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'figs/velo_x.png');


%% Animation

addpath(genpath('f_animation'));
if ~exist('figs', 'dir'); mkdir('figs'); end

[~, ~, ~, ~, ~, lo, ~, ~, ~] = model_params;
wheel_radius = 0.35;

% ====== gif ======
save_gif_switch = false ; % gif button
gif_filename = 'figs/animation.gif';
% ===================

figure('Position', [300, 300, 1800, 220]);
xlim([-5, 20]);
ylim([-0.1, 3.5]);
grid on;
xlabel('X[m]');
ylabel('Y[m]');
hold on;

% ground 
ground_patch = patch([-20,-20,20,20],[0,-0.5,-0.5,0],[0.8,0.8,0.8]);

% init the components
[wheel_rim, wheel_spokes, wheel_hub] = init_wheel(x(1), y(1), wheel_radius);
m1 = plot(x(1)+l(1)*sin(theta(1)), y(1)+l(1)*cos(theta(1)), 'o', 'MarkerSize', 20, 'MarkerFaceColor', [0.6,0.8,1]);
m2 = plot(x(1), y(1), 'o', 'MarkerSize', 12, 'MarkerFaceColor', [0.6,0.8,1]);
[spring_line, fixed_points] = init_spring(x(1)+l(1)*sin(theta(1)), y(1)+l(1)*cos(theta(1)), x(1), y(1));

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
    update_spring(spring_line, fixed_points, x(i)+l(i)*sin(theta(i)), y(i)+l(i)*cos(theta(i)), x(i), y(i));
    set(m1, 'XData', x(i)+l(i)*sin(theta(i)), 'YData', y(i)+l(i)*cos(theta(i)));
    set(m2, 'XData', x(i), 'YData', y(i));
    update_wheel(wheel_rim, wheel_spokes, wheel_hub, x(i)+l(i)*sin(theta(i)), y(i)+l(i)*cos(theta(i)), wheel_radius, -phi(i));
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


