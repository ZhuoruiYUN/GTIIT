function [  ] = animateSlip(obj)

figure(10)
ylim([-0.1,1.5]);
xlim([-0.3,3]);
grid on;
axis equal;
title('SLIP jumping robots Animation');

ground_patch = patch([-50,-50,50,50],[0,-10,-10,0],[0.8,0.8,0.8]);

leg_patch = patch(obj.x_body(1) + [0.01,0.01,-0.01,-0.01]*cos(obj.phi(1)) + obj.L(1)*[0,1,1,0]*sin(obj.phi(1)),...
                  obj.y_body(1) - [0.01,0.01,-0.01,-0.01]*sin(obj.phi(1)) + obj.L(1)*[0,-1,-1,0]*cos(obj.phi(1)),'k');

body_patch = patch(obj.x_body(1) + 0.1*sin(0:0.1:2*pi),obj.y_body(1) + 0.1*cos(0:0.1:2*pi),'r');

rectangle('Position',[0.15 0 0.1 0.1],'edgecolor','k','facecolor','g','linewidth',1.8);
rectangle('Position',[0.45 0 0.2 0.1],'edgecolor','k','facecolor','g','linewidth',1.8);
rectangle('Position',[1.1 0 0.1 0.2],'edgecolor','k','facecolor','g','linewidth',1.8);
rectangle('Position',[1.5 0 0.2 0.2],'edgecolor','k','facecolor','g','linewidth',1.8);


ylim([-0.1,1.5]);
xlim([-0.3,3]);
ylabel("Y[m]");
xlabel("X[m]");
%Loop through the data updating the graphics
for i = 1:length(obj.t)
    body_patch.Vertices = [obj.x_body(i) + 0.1*sin(0:0.1:2*pi);obj.y_body(i) + 0.1*cos(0:0.1:2*pi)]';
    
    leg_patch.Vertices = [obj.x_body(i) + [0.01,0.01,-0.01,-0.01]*cos(obj.phi(i)) + obj.L(i)*[0,1,1,0]*sin(obj.phi(i));...
                          obj.y_body(i) + [0.01,0.01,-0.01,-0.01]*sin(obj.phi(i)) + obj.L(i)*[0,-1,-1,0]*cos(obj.phi(i))]';
    drawnow;
end

