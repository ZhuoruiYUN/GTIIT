from numpy import cos, sin, deg2rad
# Problem 1
M_Nl = 788
M_fl = 304
c = 212e-3
b = 32e-3
r = 150e-3
f = 0.32 #coefficient of friction
pa_ref = 1000e3
F = (M_Nl + M_fl )/ c
print(f"The actuate force is : {F:.4}")
#problem2
M_Nr = M_Nl/1000
M_fr = M_fl/1000
pa = c * F / (M_Nr - M_fr)
print(f"the maximum pressure between right hand side shoe and rim is {pa :.4}") 
#problem3
theta_1 = deg2rad(0)
theta_2 = deg2rad(126)
theta_a = deg2rad(90)
T_L = (f * pa_ref * b * r**2 * (cos(theta_1) - cos(theta_2)) )/ sin(theta_a)
T_R = (f * pa * b * 1000 * r**2 * (cos(theta_1) - cos(theta_2)) )/ sin(theta_a)
T_tot = T_R + T_L
print(f"The breaking capacity is total torque : {T_tot:.7}")





