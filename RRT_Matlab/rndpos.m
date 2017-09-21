function point = rndpos(start, goal, x_max, y_max,i)

%-----------------------Create an ellipse---------------------------------%

% Center of the Ellipse. The start and the goal point are the foci of the
% ellipse and the mif-point of them is the center of the ellipse
xCenter = (start(1)+goal(1))/2;
yCenter = (start(2)+goal(2))/2;

% They are the length of the semi-major axis and semi-minor axis.
% x-radius  - Elongation in x-axis
% y-radius  - Elongation in y-axis
xRadius = 10;
yRadius = 7;

% The points in the ellipse are
theta = 0 : 0.01 : 2*pi;
x_elps = xRadius * cos(theta) + xCenter;
y_elps = yRadius * sin(theta) + yCenter;


%-----------------------Generate Random point-----------------------------%
    flag = 0;
    while (flag == 0); 
        % Generate a random point based upon the boundry conditions
         x_rand = rand(1)*x_max;
         y_rand = rand(1)*y_max;
         
        % Check if the random point generated is with in the ellipse boundry
        % or not, and continue generating new point until satisfy the being
        % with in the ellipse
            if (inpolygon(x_rand, y_rand, x_elps, y_elps))
                point = [x_rand, y_rand];
                flag = 1;
            end
    end


end
