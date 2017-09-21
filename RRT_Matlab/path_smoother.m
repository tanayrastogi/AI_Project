function newpath = path_smoother(path)

%---------------------------Reading Map from Json-------------------------%

% Read Map data from the Json file
map_data = loadjson('problem_B.json');

% Boundry 
dims = map_data.boundary_polygon;
x_dims = dims(:,1);
y_dims = dims(:,2);
x_max = max(x_dims);
y_max = max(y_dims);


% Obstacles
ob1 = map_data.polygon0;
x_ob1 = ob1(:,1);
y_ob1 = ob1(:,2);

ob2 = map_data.polygon1;
x_ob2 = ob2(:,1);
y_ob2 = ob2(:,2);

ob3 = map_data.polygon2;
x_ob3 = ob3(:,1);
y_ob3 = ob3(:,2);

ob4 = map_data.polygon3;
x_ob4 = ob4(:,1);
y_ob4 = ob4(:,2);

ob5 = map_data.polygon4;
x_ob5 = ob5(:,1);
y_ob5 = ob5(:,2);

ob6 = map_data.polygon5;
x_ob6 = ob6(:,1);
y_ob6 = ob6(:,2);

%---------------------------Start and Goal--------------------------------%
% Start position
q_start = map_data.start_pos;

% Goal position
q_goal = map_data.goal_pos;
%---------------------------Plotting of Map-------------------------------%
figure(2)
% Plotting obstacles
fill(x_ob1,y_ob1, 'g');
hold on
fill(x_ob2,y_ob2, 'g');
hold on
fill(x_ob2,y_ob2, 'g');
hold on
fill(x_ob3,y_ob3, 'g');
hold on
fill(x_ob4,y_ob4, 'g');
hold on
fill(x_ob5,y_ob5, 'g');
hold on
fill(x_ob6,y_ob6, 'g');
hold on
% Plotting start point
plot(q_start(1), q_start(2), '*','LineWidth', 2);
% Plotting goal point
plot(q_goal(1), q_goal(2), 'o','LineWidth', 2);
axis([0 x_max+5 0 y_max+5]);
hold on
% Plotting boundry
plot(x_dims,y_dims);
hold on

% Plotting original path
    plot(path(:,1), path(:,2),'Color', 'r');

%---------------------------Path Smoothning-------------------------------%
p = smooth(path);
len_p = size(p,1);
half_p = round(len_p/2);

newpath = [p(1:half_p,:), p(half_p+1:len_p,:)];

% Plotting the new path
plot(newpath(:,1), newpath(:,2),'Color', 'b');

% Check if the path is colliding with any obstacle or not
% Check for boundry
flag = 0;
while flag == 0
    x = newpath(:,1);
    y = newpath(:,2);
    % Check for intersection
    [xi, yi, obs, flag] = intersection(x, y, ob1, ob2, ob3, ob4, ob5, ob6);
    
    % If no intersection break the loop 
    if flag == 1
        break;
    end
    
    % Intersection point
    plot(xi, yi,'o')   
    
    col_point = [xi, yi];
    disp(obs);
    
    for k = 1:size(col_point,1)
        ndist = [];
        % For each point in the collision
            for j = 1:length(newpath)
                tmp = dist(newpath(j,:), col_point(k,:));
                ndist = [ndist tmp];
            end
            [val, idx] = min(ndist);
            disp(val);
            disp(idx);
        % Adjusting the intersecting point in the newpath outside to the obstacle        
        newpath(idx,:) = (newpath(idx,:)+path(idx,:))/2;        
        newpath(idx+1,:) = (newpath(idx+1,:)+path(idx+1,:))/2;
        newpath(idx-1,:) = (newpath(idx-1,:)+path(idx-1,:))/2;
    end
    
    % Plotting the new path
    plot(newpath(:,1), newpath(:,2),'Color', 'b');

end

end

