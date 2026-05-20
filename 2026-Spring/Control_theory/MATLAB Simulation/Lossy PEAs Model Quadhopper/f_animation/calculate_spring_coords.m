function [springX, springY] = calculate_spring_coords(x1, y1, x2, y2, numLoops, width, resolution)
    
    dx = x2 - x1;
    dy = y2 - y1;
    L = sqrt(dx^2 + dy^2); 
    theta = atan2(dy, dx); 
    
    
    t = linspace(0, numLoops*2*pi, resolution*numLoops);
    
    
    localX = linspace(0, L, length(t));
    localY = width * sin(t);
    
    
    springX = x1 + localX*cos(theta) - localY*sin(theta);
    springY = y1 + localX*sin(theta) + localY*cos(theta);
    
    
    startX = [x1, springX(1)];
    startY = [y1, springY(1)];
    endX = [springX(end), x2];
    endY = [springY(end), y2];
    
    
    springX = [startX, springX, endX];
    springY = [startY, springY, endY];
end