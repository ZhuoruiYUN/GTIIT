A = readmatrix("yunzhuorui/GTIIT/2026-Spring/Control_theory/Homework/Homework4/HW4_A.xlsx");
B = [1;0;2;0];
C = [0 1 0 3];
D = 0;

AA = [A B; C 0] ;
bb = [0; 0; 0; 0; 1] ;
%% estimate the system is controllable
Co = ctrb(A, B);
if rank(Co) == size(A, 1)
    disp('System is controllable. Proceeding to pole placement.')
else
    error('System is NOT controllable. Pole placement is not possible.')
end

%% Franklin eq.(7.97) to (7.98)
pole1 = -5 + 4*1j ;
pole2 = -5 - 4*1j ;
pole3 = -8 ;
pole4 = -22 ;
K = acker(A,B,[pole1,pole2,pole3,pole4]);
xx = AA \ bb ;

display(["Feedback gain matrix is:", K]) ;

%% (b) 

Nx = xx(1:4) ;
Nu = xx(5) ;
display(Nx);
display(Nu);
Nbar = Nu + K * Nx ;
display(Nbar)

ss_sys = ss(A-B*K, B*Nbar, C-D*K ,D*Nbar) ;
display(ss_sys) ;

tt = linspace(0,4.5,400) ;
yy = step(ss_sys,tt);

simout = sim(Homework4_sim); % from external simulink


figure(1);
plot(tt+1 ,yy);
hold on
t1 = simout.ScopeData.time ;
y1 = simout.ScopeData.signals.values ;
plot(t1(1:end),y1(1:end),'r.') ;
hold off
grid on
xlabel('time (s)'); 
ylabel('y(t) = x_1(t)');




