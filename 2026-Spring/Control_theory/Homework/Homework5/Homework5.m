%% Homework5
% Parameters are the same as HW4
A = readmatrix("yunzhuorui/GTIIT/2026-Spring/Control_theory/Homework/Homework4/HW4_A.xlsx");
B = [1;0;2;0];
C = [0 1 0 3];
D = 0;
n = size(A,1);   % 4 states

AA = [A B; C 0] ;
bb = [0; 0; 0; 0; 1] ;

%% Results from HW4(given) 
K  = [14.594417419680658, 0.999096975937301, -1.297208709840328, 9.145289774526315];
Nu = 2.333388366077816;
Nx = [-0.224863793957405; -0.195311210170051; 0.629464531396181; 0.398437070056684];

Nbar = Nu + K * Nx ;
display(Nbar)

%% PART (a) – Estimator (Observer) Gain Design
fprintf('=== PART (a): Estimator Design ===\n');
 
est_poles = [-12, -14, -18, -20];
% Check observability
Ob = obsv(A, C);
fprintf('Observability matrix rank: %d (should be %d)\n', rank(Ob), n);
% Compute estimator gain L via pole placement  (place A-LC poles)

L = acker(A', C', est_poles)';
 
fprintf('Estimator gain matrix L =\n');
disp(L);
 
%% PART (b) 
Acl_b = [A,       -B*K;
         L*C,   A-L*C-B*K];

Bcl_b = [B*Nbar;  
         B*Nbar];  

Ccl_b = [C, zeros(1,n)];
Dcl_b = 0;

ss_sys = ss(Acl_b, Bcl_b,Ccl_b, Dcl_b);

t = linspace(0, 5, 400);
[y_b, t_b] = lsim(ss_sys, ones(size(t)), t);

figure;
plot(t_b, y_b, 'b', 'LineWidth', 2);
hold on;
yline(1, 'k--', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Output y(t)');
title('Part (b): Unit Step Response with Autonomous Estimator');
legend('y(t)', 'Reference r = 1');
grid on;

%% PART (c) – Internal Model Controller (Full State, No Estimator)
A_ic = [0,          C;
        zeros(n,1), A];
B_ic = [0; B];

% Desired poles: HW4 poles + new pole at s = -10
desired_poles_c = [-5+4i, -5-4i, -8, -22, -10];

% Check controllability
Q = ctrb(A_ic, B_ic);

fprintf('Augmented controllability rank: %d (should be %d)\n', rank(Q), n+1);
rankQ = rank(Q);

% Pole placement
K_ic = acker(A_ic, B_ic, desired_poles_c);
fprintf('IMC gain Kc = [K_x | K_i] =\n'); disp(K_ic);

eig(A_ic - B_ic * K_ic);

Acl_c = A_ic - B_ic * K_ic;
Bcl_c = [-1; zeros(n,1)];    % input enters through integrator
Ccl_c = [0, C];
Dcl_c = 0;
ss_c = ss(Acl_c, Bcl_c, Ccl_c, Dcl_c);
[y_c, t_c, x_c] = lsim(ss_c, ones(size(t)), t);

% ── Figure 1 comarasion with HW4 Part(b) ──────────────────────────────
% HW4 Part(b): full state feedback, u = -Kx + Nu*r
sys_hw4b = ss(A - B*K, B*Nbar, C, 0);
[y_hw4b, ~] = lsim(sys_hw4b, ones(size(t)), t);

figure;
plot(t,    y_hw4b, 'r--', 'LineWidth', 2); hold on;
plot(t_c,  y_c,   'b',   'LineWidth', 2);
yline(1, 'k--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Output y(t)');
title('Part (c): IMC vs HW4 Part (b)');
legend('HW4 Part (b) – State Feedback (no IMC)', ...
       'HW5 Part (c) – Internal Model Controller', ...
       'Reference r = 1');
grid on;

% ── Figure 2：response of all plant statas ──────────────────────────────────
% x_c 的列: [x1, x2, x3, x4, xi],
figure;
colors = lines(4);
for k = 1:n
    plot(t_c, x_c(:, k+1), 'Color', colors(k,:), 'LineWidth', 2); hold on;
end
xlabel('Time (s)'); ylabel('State value');
title('Part (c): Plant State Responses (Unit Step)');
legend('x_1','x_2','x_3','x_4', 'Location','best');
grid on;