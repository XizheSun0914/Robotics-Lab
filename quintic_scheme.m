% This function generates a trajectory using the quintic polynomial scheme
function [d,v,a]=quintic_scheme(L1,L2,theta_0,theta_f,tf,step)

n=length(theta_0); 
for i=1:n
a0=theta_0(i);
a1=0;
a2=0;
a3=(20*theta_f(i)-20*theta_0(i))/(2*tf^3);
a4=(30*theta_0(i)-30*theta_f(i))/(2*tf^4);
a5=(12*theta_f(i)-12*theta_0(i))/(2*tf^5);

t=0:step:tf;
d=a0+a1.*t+a2.*t.^2+a3.*t.^3+a4.*t.^4+a5.*t.^5;
v=a1+2.*a2.*t+3.*a3.*t.^2+4.*a4.*t.^3+5.*a5.*t.^4;
a=2.*a2+6.*a3.*t+12.*a4.*t.^2+20.*a5.*t.^3;
Tjj(i,:)=d;
hold on;
figure(2)
plot(t,d)
title('Time vs Displacement','fontsize', 14)
xlabel('t (s)','fontsize', 12)
ylabel('d (deg)','fontsize', 12)
if i==1
    text(1,a0+a1+a2+a3+a4+a5,'Joint 1')
else
    text(1,a0+a1+a2+a3+a4+a5,'Joint 2')
end
axis square
hold on;
figure(3)
plot(t,v)
title('Time vs Velocity','fontsize', 14)
xlabel('t (s)','fontsize', 12)
ylabel('v (deg/s)','fontsize', 12)
if i==1
    text(1,a1+2*a2+3*a3+4*a4+5*a5,'Joint 1')
else
    text(1,a1+2*a2+3*a3+4*a4+5*a5,'Joint 2')
end
axis square
hold on;
figure(4)
plot(t,a)
title('Time vs Acceleration','fontsize', 14)
xlabel('t (s)','fontsize', 12)
ylabel('a (deg/s^2)','fontsize', 12)
if i==1
    text(1,2*a2+6*a3+12*a4+20*a5,'Joint 1')
else
    text(1,2*a2+6*a3+12*a4+20*a5,'Joint 2')
end
axis square
end
FK_test2(Tjj*pi/180,L1,L2)