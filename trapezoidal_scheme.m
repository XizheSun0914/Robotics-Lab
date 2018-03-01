% This function generates a trajectory using the trapezoidal scheme
function [d,v,a]=trapezoidal_scheme(L1,L2,theta_0,theta_f,tf,step,blend)

n=length(theta_0);
for i=1:n
 
    alpha_b=4*(theta_f(i)-theta_0(i))/tf^2*blend;
    tb=min(roots([alpha_b -alpha_b*tf theta_f(i)-theta_0(i)]));

    
    d=[]; v=[]; a=[]; T=[];
    
    %phases
    for t=0:step:tf
    if (t>=0) & (t<=tb)
        d=[d theta_0(i)+.5*alpha_b*t^2];
        v=[v alpha_b*t];
        a=[a alpha_b]; 
        T=[T t];
    end
   
    if (t>tb) & (t<=(tf-tb))
        d=[d theta_0(i)+alpha_b*tb*(t-tb/2)];
        v=[v alpha_b*tb];
        a=[a 0];
        T=[T t];
    end
    if (t>(tf-tb)) & (t<=tf)
        d=[d theta_f(i)-.5*alpha_b*(tf-t)^2];
        v=[v alpha_b*(tf-t)];
        a=[a -alpha_b];
        T=[T t];
    end
end

Tjj(i,:)=d;
hold on;
figure(2)
plot(T,d)
title('Time vs Displacement','fontsize', 14)
xlabel('t (s)','fontsize', 12)
ylabel('d (deg)','fontsize', 12)
if i==1
    text(tb,theta_0(i)+.5*alpha_b*tb^2,'Joint 1')
else
    text(tb,theta_0(i)+.5*alpha_b*tb^2,'Joint 2')
end
axis square
hold on;
figure(3)
plot(T,v)
title('Time vs Velocity','fontsize', 14)
xlabel('t (s)','fontsize', 12)
ylabel('v (deg/s)','fontsize', 12)
if i==1
    text(tb,alpha_b*tb,'Joint 1')
else
    text(tb,alpha_b*tb,'Joint 2')
end
axis square
hold on;
figure(4)
plot(T,a)
title('Time vs Acceleration','fontsize', 14)
xlabel('t (s)','fontsize', 12)
ylabel('a (deg/s^2)','fontsize', 12)
if i==1
    text(tb,alpha_b,'Joint 1')
else
    text(tb,alpha_b,'Joint 2')
end
axis square
end
FK_test2(Tjj*pi/180,L1,L2);