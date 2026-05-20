close all; clear all; clc

syms m1 m2 Jm Jw Jmot lo k b g 
syms x y phi theta l;
syms dx dy dphi dtheta dl;

% define the coordinates
q = [x; y; theta; l; phi];
dq = [dx; dy; dtheta; dl; dphi];
q_number = length(q);

% set the unit vector
e1 = [1; 0];
e2 = [0; 1];

% define the position vector
r_m1 = (x + l * sin(theta)) * e1 + (y + l * cos(theta)) * e2;
r_m2 =  x * e1 + y * e2;

% define the velocity vector
v_m1 = jacobian(r_m1, q) * dq;
v_m2 = jacobian(r_m2, q) * dq;

% kinetic energy
T = 1 / 2 * m1 * (v_m1.' * v_m1) + ...
    1 / 2 * m2 * (v_m2.' * v_m2) + ...
    1 / 2 * Jm * dtheta^2 + ...
    1 / 2 * Jw * (dtheta + dphi)^2 + ... 
    1 / 2 * Jmot * dl^2;
T = simplify(T);

% potential energy
V = m1 * g * r_m1.' * e2 + ...
    m2 * g * r_m2.' * e2 + ...
    1 / 2 * k * (lo - l)^2;
V = simplify(V);

% dissipation function
D = 1 / 2 * b * dl^2;
D = simplify(D);

%% M(q)
% calculate M(q) = ∂^2 T / ∂q_dot^2
M_q = sym(zeros(q_number,q_number)); 

for i = 1:q_number
    for j = 1:q_number
        % calculate first order derivative
        dT_q_dot = diff(T, dq(i));  
        % calculate second order derivative
        M_q(i,j) = diff(dT_q_dot, dq(j));  
    end
end
% output M(q)
print_matrix('M', simplify(M_q));

%% Fd(q)
dFd_q = sym(zeros(q_number, 1));  
for i = 1:q_number
    dFd_q(i) = diff(D, dq(i));
end

%% B(q,dq)
B_q_q_dot = sym(zeros(q_number, 1)); 

for i = 1:q_number
    for j = 1:q_number
        dT_q_dot_q = diff(diff(T, dq(i)), q(j)); 
        B_q_q_dot(i,j) = dT_q_dot_q;
    end
end

dT_q = sym(zeros(q_number, 1));  
for i = 1:q_number
    dT_q(i) = diff(T, q(i));  
end

% B(q, dq) = ∂^2 T / ∂q_dot ∂q * dq - ∂T / ∂q
B_q_q_dot_q_dot = B_q_q_dot * dq;
B = B_q_q_dot_q_dot + dT_q;

print_matrix('B', simplify(B+dFd_q));

%% G(q)
dV_q = sym(zeros(q_number, 1)); 
for i = 1:q_number
    dV_q(i) = diff(V, q(i)); 
end

print_matrix('G', simplify(dV_q));

