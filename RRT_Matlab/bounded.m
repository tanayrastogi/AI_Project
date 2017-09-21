function flag = bounded(q_new, q_near, bnd, ob1, ob2, ob3, ob4, ob5, ob6)
% Initialize
flag = 0;
%---------------------------Obstacles and Boundry-------------------------%
% x and y of the boundry
x_bnd = bnd(:,1);
y_bnd = bnd(:,2);

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

%---------------------------Check for collision---------------------------%
% x and y cordinates of random point
x_unit = (q_new(1)-q_near(1))/dist(q_new,q_near);
y_unit = (q_new(2)-q_near(2))/dist(q_new,q_near);

x_q_rand = x_unit;
y_q_rand = y_unit;

%---------------------------Check for collision---------------------------%

    % Check if the point is inside a polygon
    if (inpolygon(x_q_rand, y_q_rand, x_bnd, y_bnd))
        % Check if the point is not inside the obstacle1
        if(inpolygon(x_q_rand, y_q_rand, x_ob1, y_ob1) == 0)
            % Check if the point is not inside the obstacle2
            if(inpolygon(x_q_rand, y_q_rand, x_ob2, y_ob2) == 0)
                % Check if the point is not inside the obstacle3
                if(inpolygon(x_q_rand, y_q_rand, x_ob3, y_ob3) == 0)
                   % Check if the point is not inside the obstacle4
                    if(inpolygon(x_q_rand, y_q_rand, x_ob4, y_ob4) == 0)
                        % Check if the point is not inside the obstacle5
                        if(inpolygon(x_q_rand, y_q_rand, x_ob5, y_ob5) == 0)
                            % Check if the point is not inside the obstacle6
                            if(inpolygon(x_q_rand, y_q_rand, x_ob6, y_ob6) == 0)
                                flag = 1;
                            else
                                flag = 0;
                            end
                        else
                            flag = 0;
                        end
                    else
                        flag = 0;
                    end
                else
                    flag = 0;
                end
            else
                flag = 0;
            end
        else
            flag = 0;
        end
    end
 end

