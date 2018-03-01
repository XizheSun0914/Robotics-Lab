% inverse.m
% This function computes the inverse kinematics of an two revolute-jointed manipulator for a
% given position of the end-effector.

% Inputs:  x, y    -- position of the end-effector 
%          L1, L2v -- Length of the links 
%          config  -- the parameter config defines the configuration of the manipulator i.e.,
%                     config = [ 1  1]  Elbow down - Elbow down 
%                     config = [-1 -1]  Elbow up - Elbow up 
%                     config = [ 1 -1]  Elbow down - Elbow up 
%                     config = [-1  1]  Elbow up - Elbow down 
% Output:   theta -- numerical values for theta 1 and theta 2 

% Written by Flavio Firmani, University of Victoria.
% Last modified: May 30, 2005.


function theta=inverse(x,y,L1,L2,config)
    c2=(x^2+y^2-L1^2-L2^2)/(2*L1*L2);
    s2=config*sqrt(1-c2^2); %the parameter config defines the configuration of the manipulator 
    theta_2=atan2(s2,c2);
    k1=L1+L2*c2;
    k2=L2*s2;
    theta_1=atan2(y,x)-atan2(k2,k1);
    theta=[theta_1, theta_2]*180/pi;
return
