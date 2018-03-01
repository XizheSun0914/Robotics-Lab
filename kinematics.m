% kinematics.m
% This function computes the forward kinematics of an n DOF manipulator for a
% given set of joint variables.

% Inputs:  tjj -- a joint variable vector of dimention n, where n denotes 
%                 the DOF of the manipulator.
%                 Causion: Make sure that if the i-th joint is revolute, 
%                          then the value of theta_w(i) is in radians 
%                           not in degrees!)
%            v -- n1-vector indicating the nature of each joint:
%                 joint i is revolute if v(i) == 1;
%                 joint i is prismatic if v(i) == 0;
%                 joint i is actually NOT an active joint, if v(i) == -1;
%           DH -- an n1 x 3 matrix mimicking the D-H table with n1 the number
%                 of frames other than frame {0}.
%                 Column 1 of DH is a_(i-1),column 2 is alpha_(i-1).
%                 In column 3:
%                 DH(i,3) represents d_i if joint i is revolute (v(i) = 1);
%                 or represents theta_i if joint i is prismatic (v(i) = 0).
%                 If v(i) = -1, then DH(i,3) represents d_i and theta_i = 0.
% Output:   FK -- the 4 x 4 homogeneous transformation matrix of the last 
%                 frame {n1} in relation to reference frame {0}.

% Written by W.-S. Lu, University of Victoria.
% Last modified: May 28, 2001.

function FK = kinematics(tjj,v,DH)

n = size(tjj)*[1 0]';
n1 = length(v);
FK = eye(4);
for i = 1:n,
   alpha = DH(i,1);
   a = DH(i,2);
   if v(i) == 1,
      d = DH(i,3);
      ang = tjj(i);
   else
      d = tjj(i);
      ang = DH(i,3);
   end
   FK = FK*T_basic(alpha,a,d,ang);
end
if n1 > n,
   alpha = DH(n1,1);
   a = DH(n1,2);
   d = DH(n1,3);
   ang = 0;
   FK = FK*T_basic(alpha,a,d,ang);
end

   
      
      
      
 



