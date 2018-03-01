function traj_pos = MakeLinearTrajectory_TaskSpace(pos,q_dot,r)
%q-----m*3matrix-----m points for 3 joints
%q_dot-----1*3vector-----speed of three joints
m = size(pos,1);
G = pos(1,:);

for i=1:1:m-1
    pos_current = pos(i,:);
    pos_next = pos(i+1,:);
    delta = pos_next-pos_current;
    pos_dot = r*q_dot;
    min_time = abs(delta./pos_dot);
    max_time = max(min_time);
    pos_dot_modified = delta/max_time;
    step_size = 0.001*pos_dot_modified;
    tra = LinearTrajectory(pos_current,pos_next,step_size);
    G=[G; tra];
end

    pos_current = pos(m,:);
    pos_next = pos(1,:);
    delta = pos_next-pos_current;
    pos_dot = r*q_dot;
    min_time = abs(delta./pos_dot);
    max_time = max(min_time);
    pos_dot_modified = delta/max_time;
    step_size = 0.001*pos_dot_modified;
    G =[G; LinearTrajectory(pos_current,pos_next,step_size)];

traj_pos = G;