% T_basic.m
% This function computes the basic homogeneous transformation 
% matrix T for a given set of D-H parameters (see Eq.(3.6) on 
% p.84 of J. Craig's book).

% Inputs:     a -- a_(i-1)
%         alpha -- alpha_(i-1)
%             d -- d_i
%           ang -- theta_i
% Output:    Tb -- the basic homogeneous transformation matrix.

% Written by W.-S. Lu, University of Victoria.
% Last modified: May 28, 2001.

function Tb = T_basic(alpha,a,d,ang)
 
Tb = eye(4);
cc = cos(ang);
sc = sin(ang);
ca = cos(alpha);
sa = sin(alpha);
Tb(1,1) = cc;
Tb(1,2) = -sc;
Tb(1,4) = a;
Tb(2,1) = sc*ca;
Tb(2,2) = cc*ca;
Tb(2,3) = -sa;
Tb(2,4) = -sa*d;
Tb(3,1) = sc*sa;
Tb(3,2) = cc*sa;
Tb(3,3) = ca;
Tb(3,4) = ca*d;
 

 
   
      
      
      
 



