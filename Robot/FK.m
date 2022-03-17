% This is the function to compute the robot forward kinematics.
% Copyright (C) 2022  
% 
% Authors:
% Ruixuan Liu: ruixuanl@andrew.cmu.edu
% Rui Chen: ruic3@andrew.cmu.edu
% Changliu Liu : cliu6@andrew.cmu.edu
% 
% This program is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public License
% as published by the Free Software Foundation; either version 2
% of the License, or (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program; if not, write to the Free Software
% Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

function Trans=FK(robot, th)
DH = robot.DH;
nlink = size(DH, 1);
DH(:, 1) = DH(:, 1) + th;
T=eye(4);
T(1:3, 4) = robot.base;

a=DH(:,4);
A=DH(:,3);
D=DH(:,2);
q=DH(:,1);

for i=1:nlink
    T=T * [cos(q(i)) -sin(q(i))*cos(a(i))  sin(q(i))*sin(a(i))  A(i)*cos(q(i));...
           sin(q(i))  cos(q(i))*cos(a(i)) -cos(q(i))*sin(a(i))  A(i)*sin(q(i));...
           0            sin(a(i))                cos(a(i))      D(i);...
           0                0                       0           1];
end
Trans = T;
end