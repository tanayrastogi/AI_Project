function createEnvironment()

%---------------------------Read Json-------------------------------------%
    % Read Map data from the Json file
    map_data = loadjson('problem_E.json');
    
%---------------------------Start and Goal Position-----------------------%
    start = [map_data.start_pos, 0.1];
    goal = [map_data.goal_pos, 0.1];
    
%------------------ Dimension of obstacle and boundry---------------------%    
    % Dimensions for boundry
    bnd = map_data.boundary_polygon;
    wall = wall_points(bnd);
    
    % Dimensions for obstacle1
    ob_1 = map_data.polygon0;
    ob1 = wall_points(ob_1);
    
    % Dimensions for obstacle2
    ob_2 = map_data.polygon1;
    ob2 = wall_points(ob_2);
    
    % Dimensions for obstacle3
    ob_3 = map_data.polygon2;
    ob3 = wall_points(ob_3);
    
    % Dimensions for obstacle4
    ob_4 = map_data.polygon3;
    ob4 = wall_points(ob_4);
    
    % Dimensions for obstacle5
    ob_5 = map_data.polygon4;
    ob5 = wall_points(ob_5);
    
    % Dimensions for obstacle6
    ob_6 = map_data.polygon5;
    ob6 = wall_points(ob_6);    
    
%------------------Intialization of RemoteApi ----------------------------%

    disp('Program started');
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
        
    prompt = 'enter';
    dummy = input(prompt);    
        
%-----------------------Creating Start position---------------------------%        
        % Create a object
        [res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'drawstart_function',[],[start],'Start',[],vrep.simx_opmode_blocking);
        % Check for errors
%         if (res==vrep.simx_return_ok)
%             fprintf('Dummy handle: %d\n',retInts(1));
%         else
%             fprintf('Start: Remote function call failed\n');
%             disp(res);
%         end
             
%-----------------------Creating Goal position----------------------------%        
        % Create a object
        [res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'drawgoal_function',[],[goal],'Goal',[],vrep.simx_opmode_blocking);
        % Check for erros
%         if (res==vrep.simx_return_ok)
%             fprintf('Dummy handle: %d\n',retInts(1));
%         else
%             fprintf('Goal: Remote function call failed\n');
%             disp(res);
%         end
        
%-----------------------Creating the Map----------------------------------%
%-----------------------------Boundry-------------------------------------%  
        for i = 1:size(bnd,1)
            [res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'drawobject_function',[],[wall(i).position,wall(i).length,wall(i).angle],'',[],vrep.simx_opmode_blocking);
%             if (res==vrep.simx_return_ok)
%                 fprintf('Object created');
%             else
%                 fprintf('Object: Remote function call failed\n');
%                 disp(res);
%             end
        end
%-----------------------------Obstacle1-----------------------------------%        
        for i = 1:size(ob_1,1)
            [res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'drawobject_function',[],[ob1(i).position,ob1(i).length,ob1(i).angle],'',[],vrep.simx_opmode_blocking);
%             if (res==vrep.simx_return_ok)
%                 fprintf('Object created');
%             else
%                 fprintf('Object: Remote function call failed\n');
%                 disp(res);
%             end
        end
        
%-----------------------------Obstacle2-----------------------------------%        
        for i = 1:size(ob_2,1)
            [res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'drawobject_function',[],[ob2(i).position,ob2(i).length,ob2(i).angle],'',[],vrep.simx_opmode_blocking);
%             if (res==vrep.simx_return_ok)
%                 fprintf('Object created');
%             else
%                 fprintf('Object: Remote function call failed\n');
%                 disp(res);
%             end
        end
%-----------------------------Obstacle3-----------------------------------%        
        for i = 1:size(ob_3,1)
            [res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'drawobject_function',[],[ob3(i).position,ob3(i).length,ob3(i).angle],'',[],vrep.simx_opmode_blocking);
%             if (res==vrep.simx_return_ok)
%                 fprintf('Object created');
%             else
%                 fprintf('Object: Remote function call failed\n');
%                 disp(res);
%             end
        end
%-----------------------------Obstacle4-----------------------------------%        
        for i = 1:size(ob_4,1)
            [res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'drawobject_function',[],[ob4(i).position,ob4(i).length,ob4(i).angle],'',[],vrep.simx_opmode_blocking);
%             if (res==vrep.simx_return_ok)
%                 fprintf('Object created');
%             else
%                 fprintf('Object: Remote function call failed\n');
%                 disp(res);
%             end
        end       
%-----------------------------Obstacle5-----------------------------------%        
        for i = 1:size(ob_5,1)
            [res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'drawobject_function',[],[ob5(i).position,ob5(i).length,ob5(i).angle],'',[],vrep.simx_opmode_blocking);
%             if (res==vrep.simx_return_ok)
%                 fprintf('Object created');
%             else
%                 fprintf('Object: Remote function call failed\n');
%                 disp(res);
%             end
        end 
%-----------------------------Obstacle6-----------------------------------%        
        for i = 1:size(ob_6,1)
            [res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'drawobject_function',[],[ob6(i).position,ob6(i).length,ob6(i).angle],'',[],vrep.simx_opmode_blocking);
%             if (res==vrep.simx_return_ok)
%                 fprintf('Object created');
%             else
%                 fprintf('Object: Remote function call failed\n');
%                 disp(res);
%             end
        end         
%-----------------------Movement of the start object----------------------%        
        % Read Csv file for path
        pos = csvread('E_T3.csv');
        % Number of points in path
        nop = size(pos,1);
        k = nop;
        % Move Start object from the current position to new position
        for i = 1:nop
            x = pos(k,1);
            y = pos(k,2);
            vel = 0;
            %dir = 0;            
            %vel = pos(k,3);
            dir = pos(k,4); 
            [res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'motion_funtion',[],[x, y, vel, dir],'',[],vrep.simx_opmode_blocking);
            k = k-1;
            pause(0.01)
%           if (res==vrep.simx_return_ok)
%                 fprintf('Object created');
%           else
%                 fprintf('Object: Remote function call failed\n');
%                 disp(res);
%           end
        end         
        
%-----------------------End of Vrep simulation----------------------------%     
%-----------------------Close connection----------------------------------%        
        % Now close the connection to V-REP:    
        vrep.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!
    
    disp('Program ended');
end
