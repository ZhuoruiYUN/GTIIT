function [springX, springY] = calculate_spring_coords(x1, y1, x2, y2, numLoops, width, resolution)
    % 计算两点间的距离和角度
    dx = x2 - x1;
    dy = y2 - y1;
    L = sqrt(dx^2 + dy^2);  % 两点间总长度
    theta = atan2(dy, dx);   % 两点连线与x轴的夹角
    
    % 弹簧参数计算
    t = linspace(0, numLoops*2*pi, resolution*numLoops);
    
    % 在局部坐标系中创建弹簧(沿x轴方向)
    localX = linspace(0, L, length(t));
    localY = width * sin(t);
    
    % 旋转并平移弹簧到正确位置和方向
    springX = x1 + localX*cos(theta) - localY*sin(theta);
    springY = y1 + localX*sin(theta) + localY*cos(theta);
    
    % 添加直线段连接端点
    startX = [x1, springX(1)];
    startY = [y1, springY(1)];
    endX = [springX(end), x2];
    endY = [springY(end), y2];
    
    % 组合所有点
    springX = [startX, springX, endX];
    springY = [startY, springY, endY];
end