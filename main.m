% main.m
% This program shows an example of a two revolute-jointed planar manipulator
% using the cubic polynomial scheme
% It is required to save the following m-files in the same directory:
% cubic_scheme.m, FK_test2.m, kinematics.m, T_basic.m

% Inputs for generating the trajectory 
%       L1 and L2.- lengths of the links
%       theta_0 and theta_f.- initial and final configurations
%           IMPORTANT make sure that the theta angles are in degrees (not in radians)
%       tf.- duration of the trajectory
%       step.- it is the step of time, i.e., t=0:step:tf
% The number of postures is given by tf/(50*step)+1

% Written by Flavio Firmani, University of Victoria.
% Last modified: June 21, 2011

close all; clear all 

%Link Lengths 
L1=1.8; L2=1.2;  %MODIFY

%Duration
tf=5;  %MODIFY
step=0.01; %MODIFY if you want more configurations being shown

%Inverse kinematics of initial configuration 
x=.2; y=.7; %MODIFY
config=1; %PICK Configuration There are two possible configurations
              %(Elbow up -1 or Elbow down 1).

theta_0=inverse(x,y,L1,L2,config);
for i=1:2; if theta_0(i)<0; theta_0(i)=theta_0(i)+360; end; end

%Inverse kinematics for final configuration  
x=-2.5; y=-.3; %MODIFY
config=1;  %PICK Configuration

theta_f=inverse(x,y,L1,L2,config);
for i=1:2; if theta_f(i)<0; theta_f(i)=theta_f(i)+360; end; end

%Select type of Motion by UNCOMMENTING the desired scheme  
%[d,v,a]=cubic_scheme(L1,L2,theta_0,theta_f,tf,step);
[d,v,a]=quintic_scheme(L1,L2,theta_0,theta_f,tf,step);
%blend=2;  %Factor of blending
%[d,v,a]=trapezoidal_scheme(L1,L2,theta_0,theta_f,tf,step,blend);
