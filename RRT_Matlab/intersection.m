function [xi, yi,obs, flag] = intersection(x, y, ob1, ob2, ob3, ob4, ob5, ob6)
% To check if there is any intersection with the obstacles or not
flag = 0;

%---------------------------Obstacles X and Y-----------------------------%
% x and y of the obstacle1
x_ob1 = ob1(:,1);
y_ob1 = ob1(:,2);

% x and y of the obstacle2
x_ob2 = ob2(:,1);
y_ob2 = ob2(:,2);

% x and y of the obstacle3
x_ob3 = ob3(:,1);
y_ob3 = ob3(:,2);

% x and y of the obstacle4
x_ob4 = ob4(:,1);
y_ob4 = ob4(:,2);

% x and y of the obstacle5
x_ob5 = ob5(:,1);
y_ob5 = ob5(:,2);

% x and y of the obstacle6
x_ob6 = ob6(:,1);
y_ob6 = ob6(:,2);

%---------------------------Check for intersection------------------------%
% Check for obstacle1
[xi, yi] = polyxpoly(x, y, x_ob1, y_ob1);
obs = 1;
    if(isempty(xi) && isempty(yi))
        % Check for obstacle2
        [xi, yi] = polyxpoly(x, y, x_ob2, y_ob2);
        obs = 2;
        
        if(isempty(xi) && isempty(yi))
            % Check for obstacle3
            [xi, yi] = polyxpoly(x, y, x_ob3, y_ob3);
            obs = 3;
            
            if(isempty(xi) && isempty(yi))
                % Check for obstacle4
                [xi, yi] = polyxpoly(x, y, x_ob4, y_ob4);
                obs = 4;
                
                if(isempty(xi) && isempty(yi))
                    % Check for obstacle5
                    [xi, yi] = polyxpoly(x, y, x_ob5, y_ob5);
                    obs = 5;
                    
                    if(isempty(xi) && isempty(yi))
                        % Check for obstacle6
                        [xi, yi] = polyxpoly(x, y, x_ob6, y_ob6);
                        obs = 6;
                        
                        if(isempty(xi) && isempty(yi))
                            flag = 1;
                        else
                            flag = 0;
                        end
                    end
                end
            end
        end
    end
end
