clearvars; clc; close all;

%% 定义系统
s = tf('s');
G = 50*(s + 15) / (s^4 + 36*s^3 + 484*s^2 + 3280*s);
disp('Plant G(s):'); disp(G)

%% Step 1: Obtain the Bode diagram of the uncompensated loop with constant gain K
K = 44;
G_adj = K * G; % 此时低频增益已经调好
T_uncompen = feedback(G_adj,1)

omega = logspace(-1, 2, 10000);
[mag_adj, phase_adj] = bode(G_adj, omega);
mag_adj = squeeze(mag_adj);
phase_adj = squeeze(phase_adj);

%% Step 2: Determine the phase margin of the gain adjusted uncompensated system
[Gm, Pm, Wcg, Wcp] = margin(G_adj);
disp(['Step 2: 初始相角裕度 PM = ', num2str(Pm), ' deg']);
disp('        (裕度不足 60 度，继续执行后续步骤)');

%% Step 3: Determine the new crossover frequency w'_c
req_PM = 60;
safety_margin = 5; % Allow for 5 deg phase lag from the compensator
target_phase = -180 + req_PM + safety_margin; % 目标相位为 -115 deg

% 找到相位正好等于 -115 度的频率，记为 w'_c
wc_prime = interp1(phase_adj, omega, target_phase);
disp(['Step 3: 锁定的新剪切频率 w''_c = ', num2str(wc_prime), ' rad/s']);

%% Step 4: Place the zero one decade below the new crossover frequency
z = wc_prime / 10;
TI = 1 / z;
disp(['Step 4: 补偿器零点 z = ', num2str(z), ' rad/s (TI = ', num2str(TI), ')']);

%% Step 5: Measure the necessary attenuation at w'_c
% 计算当前 G_adj 在新剪切频率处的幅值 (线性倍数)
mag_wc_prime = interp1(omega, mag_adj, wc_prime);
% 计算需要的衰减量 (dB)
attenuation_dB = -20 * log10(mag_wc_prime);
disp(['Step 5: 在 w''_c 处需要的衰减量为 ', num2str(attenuation_dB), ' dB']);

%% Step 6: Calculate alpha
% 滞后网络引入的衰减是 -20*log(alpha)，所以 alpha 也就是此时的线性幅值倍数
% -20*log10(alpha) = attenuation_dB = -20*log10(mag_wc_prime)
alpha = mag_wc_prime; 
disp(['Step 6: 计算得到 alpha = ', num2str(alpha)]);

%% Step 7: Calculate the pole
p = 1 / (alpha * TI); % 也就是 p = z / alpha
z = p * alpha
disp(['Step 7: 补偿器极点 p = ', num2str(p), ' rad/s']);
disp(['Step 7_: 补偿器零点 z = ', num2str(z), ' rad/s']);


%% Step 8: Check design and iterate
% 构建滞后网络 D_lag(s) = (s/z + 1) / (s/p + 1)
% 这种写法的妙处在于 D_lag(0) = 1，它完全不会破坏我们在 Step 1 设好的静态增益 K=44
D_lag = (s/z + 1) / (s/p + 1);
% 最终的完整控制器 Dc(s) 是 静态增益 K 乘以 滞后网络
Dc = K * D_lag; 

% 系统闭环并验证
L_comp = Dc * G;
T_comp = feedback(L_comp, 1);

disp(' ');
disp('--- Step 8: 最终验证 ---');
[Gm_c, Pm_c, Wcg_c, Wcp_c] = margin(L_comp);
disp(['实际 Phase Margin = ', num2str(Pm_c), ' deg (要求 >= 60)']);

info = stepinfo(T_comp);
disp(['实际 Settling Time = ', num2str(info.SettlingTime), ' s (要求 < 3)']);

bw_actual = bandwidth(T_comp);
disp(['实际 Bandwidth = ', num2str(bw_actual), ' rad/s (要求 <= 15)']);

% 稳态误差验证 (s*G_adj(s) 的低频极限为 K * 50*15/3280)
Kv_actual = K * (50 * 15) / 3280; 
disp(['实际 Kv = ', num2str(Kv_actual), ' (要求 >= 10, ess <= 0.1)']);

%% 绘图
figure('Name', 'Bode Diagram', 'Color', 'w');
margin(L_comp);
title('Bode Diagram of Compensated System');

figure('Name', 'Step Response', 'Color', 'w');
step(T_comp);
hold on;
yline(1.02, 'r--'); yline(0.98, 'r--');
grid on;
hold on;
step(T_uncompen)
title('Unit Step Response of Compensated System');