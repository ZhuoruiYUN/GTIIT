function update_wheel(wheel_rim, wheel_spokes, wheel_hub, x, y, radius, theta)
% UPDATE_WHEEL
    
    % update wheel position
    rim_theta = linspace(0, 2*pi, 100);
    set(wheel_rim, 'XData', x + radius*cos(rim_theta), ...
                  'YData', y + radius*sin(rim_theta));
    
    % update angle
    for i = 1:4
        angle = (i-1)*pi/2 + theta; 
        set(wheel_spokes(i), ...
            'XData', [x, x + radius*cos(angle)], ...
            'YData', [y, y + radius*sin(angle)]);
    end
    
    % update hub position
    set(wheel_hub, 'XData', x, 'YData', y);
end