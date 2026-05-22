A = readmatrix("HW6_A.xlsx");
B = [1;0;2;0];
C = [0 1 0 3];
D = 0;

AA = [A B; C 0] ;
bb = [0; 0; 0; 0; 1] ;

%% --- (a) Regulator and Estimator Design ---
% NOTE: Adjust these poles depending on the eigenvalues of the actual A matrix.
P_c = [-3, -4, -5, -6];       % Desired regulator poles
P_e = [-15, -20, -25, -30];   % Desired estimator poles (faster)
K = place(A, B, P_c);
L = place(A', C', P_e)';