function point = rndpos_cir(node, start, goal, x_max, y_max,i)

%-----------------------Create an ellipse---------------------------------%

% Center of the Ellipse. The start and the goal point are the foci of the
% ellipse and the mif-point of them is the center of the ellipse
xCenter = (start(1)+goal(1))/2;
yCenter = (start(2)+goal(2))/2;

% They are the length of the semi-major axis and semi-minor axis.
% x-radius  - Elongation in x-axis
% y-radius  - Elongation in y-axis
xRadius = 7;
yRadius = 7;

% The points in the ellipse are
theta = 0 : 0.01 : 2*pi;
x_elps = xRadius * cos(theta) + xCenter;
y_elps = yRadius * sin(theta) + yCenter;

%-----------------------Create a Circle---------------------------------%
x_rad = node(1);
y_rad = node(2);
theta = 0:0.01:2*pi;

% Increase the radius at every iteration
rad = 2 + 0.001*i ;

% Circle
x_cir = rad* cos(theta) + x_rad;
y_cir = rad* sin(theta) + y_rad;

plot(x_cir, y_cir);

%-----------------------Generate Random point-----------------------------%
        % Generate a random point based upon the boundry conditions
         x_rand = rand(1)*x_max;
         y_rand = rand(1)*y_max;
         
%-----------------------Check if it is in circle--------------------------%         
        % Check if the random point generated is with in the circle boundry
            if (inpolygon(x_rand, y_rand, x_cir, y_cir))
                point = [x_rand, y_rand];
            else
                % Create a line between the random point and the node and
                % check where they intersect
                x_line = [x_rand, node(1)];
                y_line = [y_rand, node(2)];
                
                [xi, yi] = polyxpoly(x_line, y_line, x_cir, y_cir);
                point = [xi, yi];
            end
end

