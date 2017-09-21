function time = time_cal(path)
%-----------------------Time calculation----------------------------------%
dis = [];
t_dis = 0;
vel = 0;
time = 0;

nop = size(path,1);
p = path(nop:-1:1,1:3);

for itr=1:(size(path,1)-1)
    tmp = dist(p(itr,1:2), p(itr+1,1:2));
    dis = [dis tmp];
    vel = vel + p(itr,3);
end

% Average Velocity
vel = vel/nop;

% Time and total distance
for itr=1:size(dis,2)
    t_dis = t_dis + dis(1,itr);
    time = time + dis(1,itr)/vel;    
end