function [wheel_rim, wheel_spokes, wheel_hub] = init_wheel(x, y, radius)
% INIT_WHEEL 
    
    % update the round
    theta = linspace(0, 2*pi, 100);
    wheel_rim = plot(x + radius*cos(theta), y + radius*sin(theta), ...
                    'k-', 'LineWidth', 4);
    
    % update the spokes
    wheel_spokes = gobjects(1,4);
    for i = 1:4
        angle = (i-1)*pi/2; 
        wheel_spokes(i) = plot([x, x + radius*cos(angle)], ...
                              [y, y + radius*sin(angle)], ...
                              'm-', 'LineWidth', 2);
    end
    
    % middle point
    wheel_hub = plot(x, y, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
end
