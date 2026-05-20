close all; clear all; clc

syms m1 m2 m3 Jmot Jwheel lo l1 l2 k b g Im
syms x(t) y(t) phi(t) theta(t) l(t) u(t) t

x = x(t);
y = y(t);
theta = theta(t);
l = l(t);
phi = phi(t);
u = u(t);

dx = diff(x, t);
dy = diff(y, t);
dtheta = diff(theta, t);
dl = diff(l, t);
dphi = diff(phi, t);
du = diff(u, t);


% define the coordinates
q = [x, y, theta, l, phi, u];
q_dot = [dx; dy; dtheta; dl; dphi; du];
n_q = length(q);

% set the unit vector
e1 = [1; 0];
e2 = [0; 1];

% define the position vector
r_m1 = x * e1 + y * e2;
r_m2 = (x - l1 * sin(theta)) * e1 + (y - l1 * cos(theta)) * e2;
r_m3 = (x - l * sin(theta)) * e1 + (y - l * cos(theta)) * e2;

% define the velocity vector
v_m1 = diff(r_m1,t);
v_m2 = diff(r_m2,t);
v_m3 = diff(r_m3,t);

% kinetic energy
T = 1 / 2 * m1 * (v_m1.' * v_m1) + ...
    1 / 2 * m2 * (v_m2.' * v_m2) + ...
    1 / 2 * m3 * (v_m3.' * v_m3) + ...
    1 / 2 * Im * (dtheta)^2 + ...  % (m1+m2+m3)'s rotation kinetic energy
    1 / 2 * Jwheel * (dtheta + dphi)^2 + ... % wheel's rotation kinetic energy
    1 / 2 * Jmot * (du^2); % linear motor's kinetic energy
T = simplify(T);

% potential energy
V = m1 * g * r_m1.' * e2 + ...
    m2 * g * r_m2.' * e2 + ...
    m3 * g * r_m3.' * e2 + ...
    1 / 2 * k * (lo + l1 + l2 + u - l)^2;
V = simplify(V);

% dissipation function
D = 1 / 2 * b * (du - dl)^2;
D = simplify(D);

%% M(q)
% calculate M(q) = ∂^2 T / ∂q_dot^2
M_q = sym(zeros(n_q,n_q)); 

for i = 1:n_q
    for j = 1:n_q
        % calculate first order derivative
        dT_q_dot = diff(T, q_dot(i));  
        % calculate second order derivative
        M_q(i,j) = diff(dT_q_dot, q_dot(j));  
    end
end
% output M(q)
disp('M(q):');
disp(simplify(M_q));

%% Fd(q)
dFd_q = sym(zeros(n_q, 1));  
for i = 1:n_q
    dFd_q(i) = diff(D, q_dot(i));
end

disp('Fd:');
disp(simplify(dFd_q));

%% B(q,q_dot)
B_q_q_dot = sym(zeros(n_q, 1)); 

for i = 1:n_q
    for j = 1:n_q
        dT_q_dot_q = diff(diff(T, q_dot(i)), q(j)); 
        B_q_q_dot(i,j) = dT_q_dot_q;
    end
end

dT_q = sym(zeros(n_q, 1));  
for i = 1:n_q
    dT_q(i) = diff(T, q(i));  
end

% B(q, q_dot) = ∂^2 T / ∂q_dot ∂q * q_dot - ∂T / ∂q
B_q_q_dot_q_dot = B_q_q_dot * q_dot;
B = B_q_q_dot_q_dot + dT_q;

disp('B(q, q_dot):');
disp(simplify(B+dFd_q));

%% G(q)
dV_q = sym(zeros(n_q, 1)); 
for i = 1:n_q
    dV_q(i) = diff(V, q(i)); 
end

disp('G(q):');
disp(simplify(dV_q));


