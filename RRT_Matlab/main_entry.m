% RRT* path planning

%---------------------------RRT* Algorithm--------------------------------%
% Vrep_pos = path_planner();
% pause();

%---------------------------Smoothning Algorithm--------------------------%
% newpath = path_smoother(Vrep_pos);

%---------------------------Motion Stage----------------------------------%
path = csvread('kinematicPoint_B.csv');
motion_model(path);


