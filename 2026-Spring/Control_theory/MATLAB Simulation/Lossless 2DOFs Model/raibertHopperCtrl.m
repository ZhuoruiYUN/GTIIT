function [obj] = raibertHopperCtrl(obj, des_vel)
% Simple Raibert hopper Controler for SLIP monoped
% This controller is called at lift off to adjust desired leg touchdown
% angle to maintain a desired forward velocity

kv = 0.05; % m/(m/s) 

if obj.dynamic_state == 0 % Entering Flight Phase

    % 0.002 [s] is prediction stance duration.
    x_foot = 0.5 * 0.2 * obj.dx_body(end)+ kv*(obj.dx_body(end) - des_vel);

    % Desired touchdown leg angle
    obj.phi_td = asin(x_foot/obj.L0);
    
end % If entering stance, no need for the controller to run

end

