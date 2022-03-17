% This is the function to compute the robot inverse kinematics.
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

function [q, status] = IK(robot,theta_init, cart_goal)
    theta_init = theta_init/180*pi;
    T_s = FK(robot, theta_init);
    pos_s = T_s(1:3, 4);
    pos_e = cart_goal;
    
    err = pos_e-pos_s;
    th = theta_init;
    epsilon = robot.ik_eps;
    step = robot.ik_step;
    idx = 1;
    max_itr = 10000;
    status = 1;

    while(abs(err(1))>epsilon || abs(err(2))>epsilon || abs(err(3))>epsilon)
        J = jacobian(robot, th);
        th = th+step*pinv(J)*err;
   
        homogeneous = FK(robot, th);
        err = pos_e-homogeneous(1:3, 4);
        idx = idx+1;
        if(idx > max_itr)
            status = 0;
            break;
        end
    end
    idx;
    q = th/pi*180;
end


