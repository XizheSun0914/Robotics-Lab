% FK_test2.m
% This function examines the kinematics of the 2 DOF (RR) 
% planar robot (same as FK_test1.m does), and generates an
% improved figure of robot configurations with a gripper 
% attached. The size of the robot: L1 = 2 m, L2 = 1 m.
% Written by W.-S. Lu, University of Victoria.
% Last modified: June 3, 2004 by Flavio Firmani.

function FK_test2(Tjj,L1,L2)

% Generate joint trajectory 
% theta_0 = [10 25]'*pi/180; 
% theta_f = [75 60]'*pi/180; 
% T = 1:4/199:5; % Time duration: [1, 5]
% Tjj = traj_j1(theta_0,theta_f,T);
[r,c]=size(Tjj);
% Generate the trajectories of the end-effector
% and the distant end of the first link.
DH = [0 0 0; 0 L1 0; 0 L2 0];
v = [1 1 -1]';
DH1 = DH(1:2,:);
v1 = [1 -1]';
for i = 1:c,
   FK3 = kinematics(Tjj(1:2,i),v,DH);
   FK2 = kinematics(Tjj(1:1,i),v1,DH1);
   R3(:,2*i-1:2*i) = FK3(1:2,1:2);
   Tj3(:,i) = FK3(1:2,4);
   Tj2(:,i) = FK2(1:2,4);
end

% Plot trajectories and robot configurations
figure(1)
plot(Tj3(1,:), Tj3(2,:),'k:')
%grid on
hold on;
q1 = [0 Tj2(1,1) Tj3(1,1)];
q2 = [0 Tj2(2,1) Tj3(2,1)];
plot(q1(1:2),q2(1:2),'.-',q1(2:3),q2(2:3),'b-') % plot 1st configuration 
Q1 = Tj3(:,1) + 0.07*R3(:,1) + 0.05*R3(:,2);
Q2 = Tj3(:,1) + 0.05*R3(:,2);
Q3 = Tj3(:,1) - 0.05*R3(:,2);
Q4 = Tj3(:,1) + 0.07*R3(:,1) - 0.05*R3(:,2);
Q = [Q1 Q2 Q3 Q4];
plot(Q(1,:),Q(2,:),'-') % plot the 1st gripper


for i = 1:(c/50),
   ii = i*50;
   q1 = [0 Tj2(1,ii) Tj3(1,ii)];
   q2 = [0 Tj2(2,ii) Tj3(2,ii)];
   plot(q1(1:2),q2(1:2),'.-',q1(2:3),q2(2:3),'b-')% plot (i*20)th configuration 
   Q1 = Tj3(:,ii) + 0.07*R3(:,2*ii-1) + 0.05*R3(:,2*ii);
   Q2 = Tj3(:,ii) + 0.05*R3(:,2*ii);
   Q3 = Tj3(:,ii) - 0.05*R3(:,2*ii);
   Q4 = Tj3(:,ii) + 0.07*R3(:,2*ii-1) - 0.05*R3(:,2*ii);
   Q = [Q1 Q2 Q3 Q4];
   plot(Q(1,:),Q(2,:),'-') % plot the (i*20)th gripper
end

% Draw a robot base and print figure title, etc.
plot(-0.25:0.5/100:0.25,zeros(1,101),'linewidth',3)
plot(-0.25:0.5/100:0.25,-0.04*ones(1,101),'linewidth',3)
axis equal
%axis([-1 4 -1 4])
text(-0.13,-0.3,'BASE')
title('Robot Configurations','fontsize', 14)
xlabel('x0','fontsize', 12)
ylabel('y0','fontsize', 12)
hold on;
