function wall = wall_points(obj)
% Function to find the center points of each side of the object and find
% how much angle is the side with x-axis

% Number of walls
n = size(obj,1);

for j = 1:n-1
    wall(j).position = (obj(j,:) + obj(j+1,:))/2;
    wall(j).length = (sqrt((obj(j+1,1) - obj(j,1))^2 + (obj(j+1,2) - obj(j,2))^2));
    wall(j).angle = atan2((obj(j+1,2) - obj(j,2)),(obj(j+1,1) - obj(j,1)));
end

    wall(n).position = (obj(n,:) + obj(1,:))/2;
    wall(n).length = (sqrt((obj(1,1) - obj(n,1))^2 + (obj(1,2) - obj(n,2))^2));
    wall(n).angle = atan2((obj(n,2) - obj(1,2)),(obj(n,1) - obj(1,1)));
    
end

