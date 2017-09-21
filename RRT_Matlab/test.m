goal = [19.0, 3.0];
start = [2.1, .1];

xCenter = (start(1)+goal(1))/2;
yCenter = (start(2)+goal(2))/2;

% xCenter = 12.5;
% yCenter = 10;

xRadius = 8;
yRadius = 2;

theta = 0 : 0.01 : 2*pi;
x = xRadius * cos(theta) + xCenter;
y = yRadius * sin(theta) + yCenter;
plot(x, y, 'LineWidth', 3);
axis square;
xlim([-5 20]);
ylim([-5 20]);
grid on;