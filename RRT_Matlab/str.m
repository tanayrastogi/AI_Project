function [flag, new] = str(q_rand, q_near, val, vel)
   % Initializing the new point
   qnew = [0 0];
   % Time
   dt = 0.1;
   
   x_unit = (q_rand(1)-q_near(1))/dist(q_rand,q_near);
   y_unit = (q_rand(2)-q_near(2))/dist(q_rand,q_near);
  
   % Kinematic Equations
    qnew(1) = q_near(1) + x_unit*vel(1)*dt;
    qnew(2) = q_near(2) + y_unit*vel(2)*dt;
    
    new = [qnew(1), qnew(2)];
    
    % Check if the point is bounded inside any polygon or not
    f = bounded(q_new, q_near, dims, ob1, ob2, ob3, ob4, ob5, ob6);
    
    if ((new == q_rand) || f == 1)
        flag = 1;
    else
        flag =0;
    end
    
end