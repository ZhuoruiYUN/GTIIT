%% Poincare
close all; clear all; clc

Z0 = [0.5; 0.3];  %  [y; l;];

%Converges after 1 iteration
numIters = 1;
Z0_periodic = zeros(2,numIters);
for i = 1:numIters
    [Z0_periodic(:,i), ~, ~, ~, jacobian] = fsolve(@(Z)(Poincare_map(Z) - Z), Z0);
    Z0 = Z0_periodic(:,i)
end

jacobian_real = jacobian + eye(size(jacobian));
eigen = eig(jacobian_real)
lambda = abs(eigen)

%% solve ode45
close all; clear all; clc

X0 = [0;   0.5;  0;  0.3;   0;   0;   % [x; y; th; l ; phi; u];
      0;   0;  0;    0;   0;   0;];  

% [~, ~, ~, ~] = u_input(0, X0, true); 
[~, ~, ~] = phi_input(0, X0, true); 

dynamics_status = 1;  % 0: ground  1: flight

t_start = 0;
t_stop = 5;
op_ground = odeset('RelTol', 1e-8, 'AbsTol', 1e-8,'Events',@events_ground);    
op_flight = odeset('RelTol', 1e-8, 'AbsTol', 1e-8,'Events',@events_flight); 
X_data = [];
t_data = [];


while abs(t_stop-t_start) > 1e-2

    if dynamics_status == 1
        tspan = t_start:0.01:t_stop;
        [t,X,~,~,ie] = ode45(@(t,X)sys_flight(t,X), tspan, X0, op_flight);
        X_data = [X_data; X];
        t_data = [t_data; t];
        t_start = t(end);
        X0 = [X(end,1); X(end,2); X(end,3); X(end,4); X(end,5); X(end,6);
              X(end,7); X(end,8); X(end,9); X(end,8); X(end,11); X(end,12)];
        dynamics_status = 0;
        disp('dynamics_status: flight');   
        phi_flag = true;
        

    elseif dynamics_status == 0

        tspan = t_start:0.01:t_stop;
        [t,X,~,~,ie] = ode45(@(t,X)sys_ground(t,X),tspan,X0,op_ground);

        X_data = [X_data; X];
        t_data = [t_data; t];
        t_start = t(end);
        X0 = [X(end,1); X(end,2); X(end,3); X(end,4); X(end,5); X(end,6);
              X(end,7); X(end,8); X(end,9); X(end,10); X(end,11); X(end,12)];
        dynamics_status = 1;
        disp('dynamics_status: ground');

    end
end

x = X_data(:, 1);
y = X_data(:, 2);
th = X_data(:, 3);
l = X_data(:, 4);
phi = X_data(:, 5);
u = X_data(:, 6);

dx = X_data(:, 7);
dy = X_data(:, 8);
dth = X_data(:, 9);
dl = X_data(:, 10);
dphi = X_data(:, 11);
du = X_data(:, 12);

%% Plot x

figure('Position', [250, 250, 800, 400]);
hold on;
plot(t_data, x, 'r','LineWidth', 2);
xlabel('Time [s]', 'Interpreter', 'latex', 'fontsize', 20);
legend('$x$','$\phi$','Interpreter','latex','fontsize',20,'location','ne')
% ylim([-0.1 0.7])
grid on;


%% Plot y 

figure('Position', [250, 250, 800, 400]);
hold on;
plot(t_data, y, 'r-*','LineWidth', 2);
plot(t_data, phi, 'b-*','LineWidth', 2);
xlabel('Time [s]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('Distance [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
% legend('$y$','$Stimulation$','Interpreter','latex','fontsize',20,'location','ne')
% ylim([-0.1 0.7])
grid on;
% saveas(gcf, 'y.png');


%% Plot l

figure('Position', [250, 250, 800, 400]);
hold on;
plot(t_data, l, 'b','LineWidth', 2);
xlabel('Time [s]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('l [$m$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'l.png');

%% Plot th

figure('Position', [250, 250, 800, 400]);
hold on;
plot(t_data, th * 180 / pi, 'b','LineWidth', 2);
xlabel('Time [s]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$\theta$ [$^\circ$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'l.png');

%% Plot phi

figure('Position', [250, 250, 800, 400]);
hold on;
plot(t_data, phi * 180 / pi, 'b','LineWidth', 2);
xlabel('Time [s]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$\phi$ [$^\circ$]', 'Interpreter', 'latex', 'fontsize', 20);
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
plot(dphi, phi, 'LineWidth', 2);
xlabel('$\phi$ [$rad$]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$\dot \phi$ [$rad/s$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'phase phi-dphi.png');

figure('Position', [250, 250, 800, 400]);
hold on;
plot(dth, th, 'LineWidth', 2);
xlabel('$\theta$ [$rad$]', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$\dot \theta$ [$rad/s$]', 'Interpreter', 'latex', 'fontsize', 20);
grid on;
saveas(gcf, 'phase th-dth.png');



%% Animation
addpath(genpath('animations'));

[~, ~, ~, ~, ~, ~, lo, l1, l2, ~, ~, ~] = model_params;
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
pole2 = plot([x(1)-l1*sin(th(1)), x(1)-(l1-u(1))*sin(th(1))], [y(1)-l1*cos(th(1)),y(1)-(l1-u(1))*cos(th(1))], '-', 'LineWidth', 10, 'Color', [1,0,0]);
pole1 = plot([x(1), x(1)-l1*sin(th(1))], [y(1),y(1)-l1*cos(th(1))], '-', 'LineWidth', 5, 'Color', [0,0,0]);
pole3 = plot([x(1)-l(1)*sin(th(1)), x(1)-(l(1)-l2)*sin(th(1))], [y(1)-l(1)*cos(th(1)), y(1)-(l(1)-l2)*cos(th(1))], '-', 'LineWidth', 5, 'Color', [0,0,0]);
m1 = plot(x(1), y(1), 'o', 'MarkerSize', 20, 'MarkerFaceColor', [0.6,0.8,1]);
m2 = plot(x(1)-l1*sin(th(1)), y(1)-l1*cos(th(1)), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.6,0.8,1]);
m3 = plot(x(1)-l(1)*sin(th(1)), y(1) - l(1) * cos(th(1)), 'o', 'MarkerSize', 12, 'MarkerFaceColor', [0.6,0.8,1]);

[spring_line, fixed_points] = init_spring(0, 0, 0.5, 0.5);

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
   
    set(pole1, 'XData', [x(i), x(i)-l1*sin(th(i))], 'YData', [y(i),y(i)-l1*cos(th(i))]);
    set(pole2, 'XData', [x(i)-l1*sin(th(i)), x(i)-(l1-u(i))*sin(th(i))], 'YData', [y(i)-l1*cos(th(i)),y(i)-(l1-u(i))*cos(th(i))]);
    set(pole3, 'XData', [x(i)-l(i)*sin(th(i)), x(i)-(l(i)-l2)*sin(th(i))], 'YData', [y(i)-l(i)*cos(th(i)), y(i)-(l(i)-l2)*cos(th(i))]);
    update_spring(spring_line, fixed_points, x(i)-(l1-u(i))*sin(th(i)), y(i)-(l1-u(i))*cos(th(i)), x(i)-(l(i)-l2)*sin(th(i)), y(i)-(l(i)-l2)*cos(th(i)));

    set(m1, 'XData', x(i), 'YData', y(i));
    set(m2, 'XData', x(i)-l1*sin(th(i)), 'YData', y(i)-l1*cos(th(i)));
    set(m3, 'XData', x(i)-l(i)*sin(th(i)), 'YData', y(i) - l(i) * cos(th(i)));
    update_wheel(wheel_rim, wheel_spokes, wheel_hub, x(i), y(i), wheel_radius, -phi(i));
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


