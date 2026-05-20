function [spring_line, fixed_points] = init_spring(x1, y1, x2, y2)
    % 弹簧参数
    numLoops = 5;       % 圈数
    width = 0.03;       % 弹簧宽度
    resolution = 10;    % 每圈线段数
    
    % 计算弹簧坐标
    [springX, springY] = calculate_spring_coords(x1, y1, x2, y2, numLoops, width, resolution);
    
    % 绘制弹簧线
    spring_line = plot(springX, springY, 'b-', 'LineWidth', 2);
    
    % 绘制固定点
    hold on;
    fixed_points(1) = plot(x1, y1, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'c');
    fixed_points(2) = plot(x2, y2, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'c');
    hold off;
end