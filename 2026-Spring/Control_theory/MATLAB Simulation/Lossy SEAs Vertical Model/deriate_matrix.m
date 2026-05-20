syms m1 m2 m3 Jmot lo l1 l2 k b g y(t) l(t) u(t) t;

% define the coordinates
q = [y(t), l(t), u(t)];
q_dot = [diff(y(t), t); diff(l(t), t); diff(u(t), t)];

% set the unit vector
e1 = [1; 0];
e2 = [0; 1];

% define the position vector
r_m1 = 0 * e1 + y(t) * e2;
r_m2 = 0 * e1 + (y(t)-l1) * e2;
r_m3 = 0 * e1 + (y(t)-l(t)) * e2;

% define the velocity vector
v_m1 = diff(r_m1,t);
v_m2 = diff(r_m2,t);
v_m3 = diff(r_m3,t);

% kinetic energy
T = 1 / 2 * m1 * (v_m1.' * v_m1) + ...
    1 / 2 * m2 * (v_m2.' * v_m2) + ...
    1 / 2 * m3 * (v_m3.' * v_m3) + ...
    1 / 2 * Jmot * ((diff(u(t), t) * diff(u(t), t)));
T = simplify(T)

% potential energy
V = m1 * g * r_m1.' * e2 + ...
    m2 * g * r_m2.' * e2 + ...
    m3 * g * r_m3.' * e2 + ...
    1 / 2 * k * (lo + l1 + l2 + u(t) - l(t))^2;
V = simplify(V)

% dissipation function
D = 1 / 2 * b * (diff(u(t), t) - diff(l(t), t))^2;
D = simplify(D)

%% M(q)
% calculate M(q) = ∂^2 T / ∂q_dot^2
M_q = sym(zeros(3,3)); 

for i = 1:3
    for j = 1:3
        % calculate first order derivative
        dT_q_dot = diff(T, q_dot(i));  
        % calculate second order derivative
        M_q(i,j) = diff(dT_q_dot, q_dot(j));  
    end
end
% output M(q)
disp('M(q):');
disp(M_q);

%% Fd(q)
dFd_q = sym(zeros(3, 1));  
for i = 1:3
    dFd_q(i) = diff(D, q_dot(i));
end

disp('Fd:');
disp(dFd_q);

%% B(q,q_dot)
B_q_q_dot = sym(zeros(3, 1)); 

for i = 1:3
    for j = 1:3
        dT_q_dot_q = diff(diff(T, q_dot(i)), q(j)); 
        B_q_q_dot(i,j) = dT_q_dot_q;
    end
end

dT_q = sym(zeros(3, 1));  
for i = 1:3
    dT_q(i) = diff(T, q(i));  
end

% B(q, q_dot) = ∂^2 T / ∂q_dot ∂q * q_dot - ∂T / ∂q
B_q_q_dot_q_dot = B_q_q_dot * q_dot;
B = B_q_q_dot_q_dot + dT_q;

disp('B(q, q_dot):');
disp(simplify(B+dFd_q));

%% G(q)
dV_q = sym(zeros(3, 1)); 
for i = 1:3
    dV_q(i) = diff(V, q(i)); 
end

disp('G(q):');
disp(simplify(dV_q));


