syms m1 m2 Jmot lo k b g y(t) l(t) t;

% define the coordinates
q = [y(t); l(t)];
q_dot = [diff(y(t), t); diff(l(t), t)];
q_number = length(q);


% set the unit vector
e1 = [1; 0];
e2 = [0; 1];

% define the position vector
r_m1 = 0 * e1 + y(t) * e2;
r_m2 = 0 * e1 + (y(t)-l(t)) * e2;


% define the velocity vector
v_m1 = diff(r_m1,t);
v_m2 = diff(r_m2,t);

% kinetic energy
T = 1 / 2 * m1 * (v_m1.' * v_m1) + ...
    1 / 2 * m2 * (v_m2.' * v_m2) + ...
    1 / 2 * Jmot * ((diff(l(t), t) * diff(l(t), t)));
T = simplify(T)

% potential energy
V = m1 * g * r_m1.' * e2 + ...
    m2 * g * r_m2.' * e2 + ...
    1 / 2 * k * (lo - l(t))^2;
V = simplify(V)

% dissipation function
D = 1 / 2 * b * diff(l(t), t)^2;
D = simplify(D)

%% M(q)
% calculate M(q) = ∂^2 T / ∂q_dot^2
M_q = sym(zeros(q_number,q_number)); 

for i = 1:q_number
    for j = 1:q_number
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
dFd_q = sym(zeros(q_number, 1));  
for i = 1:q_number
    dFd_q(i) = diff(D, q_dot(i));
end

disp('Fd:');
disp(dFd_q);

%% B(q,q_dot)
B_q_q_dot = sym(zeros(q_number, 1)); 

for i = 1:q_number
    for j = 1:q_number
        dT_q_dot_q = diff(diff(T, q_dot(i)), q(j)); 
        B_q_q_dot(i,j) = dT_q_dot_q;
    end
end

dT_q = sym(zeros(q_number, 1));  
for i = 1:q_number
    dT_q(i) = diff(T, q(i));  
end

% B(q, q_dot) = ∂^2 T / ∂q_dot ∂q * q_dot - ∂T / ∂q
B_q_q_dot_q_dot = B_q_q_dot * q_dot;
B = B_q_q_dot_q_dot + dT_q;

disp('B(q, q_dot):');
disp(simplify(B+dFd_q));

%% G(q)
dV_q = sym(zeros(q_number, 1)); 
for i = 1:q_number
    dV_q(i) = diff(V, q(i)); 
end

disp('G(q):');
disp(simplify(dV_q));


