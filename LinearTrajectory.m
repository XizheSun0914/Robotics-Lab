function trajectory = LinearTrajectory(q_current,q_next,step_size)
%q_current-----1*3vector-----joint positions at the starting point
%q_next-----1*3vector-----joint positions at the next point
%step_size-----1*3vector-----the size of each step from q_current to q_next
%trajectory-----n*3matrix-----n points between q_current and q_next
num = fix(min(abs(q_next-q_current)./abs(step_size)));
mol = min(abs(q_next-q_current)./step_size)-num;
if mol ==0
    tra = zeros(num,3);
else
    tra = zeros(num+1,3);
end
for i=1:1:num
    tra(i,:) = q_current+i*step_size;
    
end
if tra(num,:)~=q_next
    tra(num+1,:) = q_next;
end
trajectory = tra;