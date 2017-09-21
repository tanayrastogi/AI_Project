function Vrep_pos = path_planner_1()
close all;
clc;

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
q_start.coord = map_data.start_pos;
q_start.cost = 0;
q_start.parent = 0;

% Goal position
q_goal.coord = map_data.goal_pos;
q_goal.cost = 0;

%---------------------------Tree Variables--------------------------------%

% Maximum number of nodes in the tree
max_nodes = 1000;

% First node in tree
nodes(1) = q_start;
q_new = nodes(1);
%---------------------------Plotting of Map-------------------------------%
% Plotting figure
figure(1)
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
plot(q_start.coord(1), q_start.coord(2), '*','LineWidth', 2);
plot(q_goal.coord(1), q_goal.coord(2), 'o','LineWidth', 2);
axis([0 x_max+3 0 y_max+3]);
hold on
plot(x_dims,y_dims);
hold on


%---------------------------RRT* Algorithm--------------------------------%
% Run for max_nodes in the tree

for i = 1:1:max_nodes
    % Generate a Random point
    q_rand = rndpos(q_start.coord, q_goal.coord, x_max, y_max,i);
%   q_rand = [rand(1)*x_max, rand(1)*y_max];
    plot(q_rand(1), q_rand(2), 'x', 'Color', 'b');
        
        % Break if goal node is already reached
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_goal.coord
                break
            end
        end
        
        % Pick the closest node from existing list to branch out from
        ndist = [];
        for j = 1:1:length(nodes)
            n = nodes(j);
            tmp = dist(n.coord, q_rand);
            ndist = [ndist tmp];
        end
        [val, idx] = min(ndist);
        q_near = nodes(idx);
        q_new = q_near;
        
        f = 0;
        while f==0
            % Steer towards the point according to the motion model
            [f, q_new.coord] = str(q_rand, q_new.coord, val, map_data.start_vel);
            plot(q_new.coord(1),q_new.coord(2),'*', 'Color', 'y'); 

            % Check if the line between the new node and the nearest does
            %  to any obstacle
%             flag = bounded(q_new.coord, q_near.coord, dims, ob1, ob2, ob3, ob4, ob5, ob6);
            if(flag)
                figure(1)
                line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
                drawnow
                hold on
                q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;

                % Within a radius of r, find all existing nodes to the current
                % node
                q_nearest = [];
                r = 1.5;
                neighbor_count = 1;
                for j = 1:1:length(nodes)
                    x1 = [nodes(j).coord(1), q_new.coord(1)];
                    y1 = [nodes(j).coord(2), q_new.coord(2)];
                    [~,~,~,flag] = intersection(x1, y1, ob1, ob2, ob3, ob4, ob5, ob6); 
                    if ((flag) && (dist(nodes(j).coord, q_new.coord) <= r))
                    q_nearest(neighbor_count).coord = nodes(j).coord;
                    q_nearest(neighbor_count).cost = nodes(j).cost;
                    neighbor_count = neighbor_count+1;
                    end
                end


                % To search for all the nodes in the tree which can lower the
                % cost of the path

                % Initialize cost to currently known value
                q_min = q_near;
                C_min = q_new.cost;

                % Iterate through all nearest neighbors to find alternate lower
                % cost paths
                for k = 1:1:length(q_nearest)
    %                 x2 = [q_nearest(k).coord(1), q_new.coord(1)];
    %                 y2 = [q_nearest(k).coord(2), q_new.coord(2)];  
    %                 [~, ~,~,flag] = intersection(x2, y2, ob1, ob2, ob3, ob4, ob5, ob6);
    %                 if (flag && (q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min))
                    if (q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min)    
                    q_min = q_nearest(k);
                    C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
                    line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');                
                    hold on
                    end
                end 
                % Update parent to least cost-from node
                for l = 1:1:length(nodes)
                    if nodes(l).coord == q_min.coord
                        q_new.parent = l;
                    end
                end

                % Append to nodes
                nodes = [nodes q_new];

            end
        end
end    

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[~, idx] = min(D);
% q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
p = 1;
    while q_end.parent ~= 0
        Vrep_pos(p,:) = [q_end.coord];
        p = p+1;
        start = q_end.parent;
        line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
        hold on
        q_end = nodes(start);
    end

end





