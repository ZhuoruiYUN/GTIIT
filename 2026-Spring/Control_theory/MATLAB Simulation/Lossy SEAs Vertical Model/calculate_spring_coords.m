function [springX, springY] = calculate_spring_coords(x1, y1, l, numLoops, width, resolution)

    springHeight = l - 0.02;
    
    
    t = linspace(0, numLoops*2*pi, resolution*numLoops);
    
    
    spiralX = x1 + width * sin(t);
    spiralY = y1 - springHeight/(numLoops*2*pi) * t;
    

    startX = [x1, spiralX(1)];
    startY = [y1, spiralY(1)];
    endX = [spiralX(end), x1];
    endY = [spiralY(end), y1 - l];
    

    springX = [startX, spiralX, endX];
    springY = [startY, spiralY, endY];
end