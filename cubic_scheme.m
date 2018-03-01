% This function generates a trajectory using the cubic polynomial scheme

% Inputs for generating the trajectory 
%       L1 and L2.- lengths of the links
%       theta_0 and theta_f.- initial and final configurations
%           IMPORTANT make sure that the theta angles are in degrees (not in radians)
%       tf.- duration of the trajectory
%       step.- it is the step of time, i.e., t=0:step:tf 


function [d,v,a]=cubic_scheme(L1,L2,theta_0,theta_f,tf,step)

n=length(theta_0);
for i=1:n
    % Determning the value of the coefficients for each joint
    a0=theta_0(i);
    a1=0;
    a2=3/tf^2*(theta_f(i)-theta_0(i));
    a3=-2/tf^3*(theta_f(i)-theta_0(i));

    % Identifying the values of displacement, velovity, and acceleration of the joints
    t=0:step:tf;
    d=a0+a1.*t+a2.*t.^2+a3.*t.^3;
    v=a1+2.*a2.*t+3.*a3.*t.^2;
    a=2.*a2+6.*a3.*t;
    Tjj(i,:)=d;

    % Plotting displacement graph
    hold on;
    figure(2)
    plot(t,d)
    title('Time vs Displacement','fontsize', 14)
    xlabel('t (s)','fontsize', 12)
    ylabel('d (deg)','fontsize', 12)
    if i==1
        text(1,a0+a1+a2+a3,'Joint 1')
    else
        text(1,a0+a1+a2+a3,'Joint 2')
    end
    axis square
    
    % Plotting velocity graph
    hold on;
    figure(3)
    plot(t,v)
    title('Time vs Velocity','fontsize', 14)
    xlabel('t (s)','fontsize', 12)
    ylabel('v (deg/s)','fontsize', 12)
    if i==1
        text(1,a1+2*a2+3*a3,'Joint 1')
    else
        text(1,a1+2*a2+3*a3,'Joint 2')
    end
    axis square
    
    % Plotting accleration graph
    hold on;
    figure(4)
    plot(t,a)
    title('Time vs Acceleration','fontsize', 14)
    xlabel('t (s)','fontsize', 12)
    ylabel('a (deg/s^2)','fontsize', 12)
    if i==1
        text(1,2*a2+6*a3,'Joint 1')
    else
        text(1,2*a2+6*a3,'Joint 2')
    end
    axis square
end
% Plotting configurations
FK_test2(Tjj*pi/180,L1,L2)