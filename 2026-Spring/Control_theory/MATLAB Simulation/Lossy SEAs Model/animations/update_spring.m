function update_spring(spring_line, fixed_points, x1, y1, x2, y2)
    % 弹簧参数
    numLoops = 5;
    width = 0.03;
    resolution = 10;
    
    % 计算新坐标
    [springX, springY] = calculate_spring_coords(x1, y1, x2, y2, numLoops, width, resolution);
    
    % 更新弹簧线
    set(spring_line, 'XData', springX, 'YData', springY);
    
    % 更新固定点
    set(fixed_points(1), 'XData', x1, 'YData', y1);
    set(fixed_points(2), 'XData', x2, 'YData', y2);
end