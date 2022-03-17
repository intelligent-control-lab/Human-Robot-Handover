% This file includes the robot properties.
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

function robot=robot_property(id)
robot.name = id;
switch id
    case 'LRMate200iD7L'
        %the constants
        robot.nlink=6;
        robot.umax=10;
        robot.margin=0.1;
        robot.ssa_margin=0.05;
        robot.delta_t=0.16; %0.05
        robot.thetamax = [170,  145,  240, 190, 125, 360;
                          -170, -100, -72, -190, -125, -360]'.*pi/180;
        robot.thetadotmax = [35,  35,  50,  50,  50,  50]'.*pi/180;
        
        %The length of each links and DH parameter and base
        robot.DH=[0,0,0.050,-1.5708;
                  -1.5708,0,0.440,3.1416;
                  0,0,0.035,-1.5708;
                  0,-0.420,0,1.5708;
                  0,0,0,-1.5708;
                  0,-0.080-0.23,0,3.1416];%theta,d,a,alpha gripper offset 0.23m.
        robot.base = [0;0;0.330+0.87];    %[0.0825   -0.0825    0.3300][0;0;0.330];%origin
        robot.ik_step = 0.1;
        robot.ik_eps = 0.01;
        robot.cap={};
        robot.cap{1}.p=[0 0;0 0;0 0];
        robot.cap{1}.r=0;
        
        robot.cap{2}.p=[-0.4 0;
                        0 0;
                        0 0];
        robot.cap{2}.r=0.13;
        
        robot.cap{3}.p=[-0.03 -0.03;0 0;0.05 0.05];
        robot.cap{3}.r=0.0;
        
        robot.cap{4}.p=[0 0;0 0.4;0 0];
        robot.cap{4}.r=0.068;
        
        robot.cap{5}.p=[0 0;
                        0 0; 
                        0.01 -0.20];% -0.32
        robot.cap{5}.r=0.1; %0.05;
        
        robot.cap{6}.p=[0.05, 0.18;
                    0 0; 0.1107 0.1107];
        robot.cap{6}.r=0.00;
        
        load('LRMate200iD7LCapsules.mat');
        robot.boundary = RoBoundary;    

    case 'KinovaGen3'
        %the constants
        robot.nlink=7;
        robot.umax=10;
        robot.margin=0.1;
        robot.ssa_margin=0.05;
        robot.delta_t=0.05;
        robot.thetamax=[100,  90,  130, 180, 120, 150, -180; 
                        -100, -90, -72, -180, -120, -150, 180]'*pi/180;
        robot.thetadotmax=[1;1.5;1.5;0.5;0.5;0.5;0.5];
        %The length of each links and DH parameter and base
        robot.DH=[0,        0.1564+0.1284,      0,      -pi/2;
                  0,        -0.0118,            0,      pi*1.5;
                  0,        -0.4208,            0,      pi/2;
                  0,        -0.0128,            0,      pi*1.5;
                  0,        -0.3143,            0,      pi/2;
                  0,        0.0,                0,      pi*1.5;
                  0,        -0.1674-0.12,       0,      pi];%theta,d,a,alpha
        robot.base=[0;0;0];%origin % contd, in python, append 0 to head of joint state for FK/IK
        robot.ik_step = 0.001;
        robot.ik_eps = 0.001; 
        robot.cap={};
        robot.cap{1}.p=[0 0;
                        0 0;
                        0 0];
        robot.cap{1}.r=0;
        
        robot.cap{2}.p=[-0.36 0;
                        0 0;
                        0 0];
        robot.cap{2}.r=0;
        
        robot.cap{3}.p=[-0.03 -0.03;
                        0 0;
                        0.05 0.05];
        robot.cap{3}.r=0;
        
        robot.cap{4}.p=[0 0;
                        0 0.35;
                        0 0];
        robot.cap{4}.r=0;
        
        robot.cap{5}.p=[0 0;
                        0 0;
                        -0.3 0.010];
        robot.cap{5}.r=0;
        
        robot.cap{6}.p=[0.05 0.17;
                        0 0; 
                        0.135 0.135];
        robot.cap{6}.r=0;

        robot.cap{7}.p=[0 0;
                        0 0; 
                        0 0.2];
        robot.cap{7}.r=0.1;
        
        %???
        robot.boundary = [];
end

%The kinematic matrices
robot.A=[eye(robot.nlink) robot.delta_t*eye(robot.nlink);
        zeros(robot.nlink) eye(robot.nlink)];
robot.B=[0.5*robot.delta_t^2*eye(robot.nlink);
        robot.delta_t*eye(robot.nlink)];
robot.Ac=[eye(3) robot.delta_t*eye(3);
        zeros(3) eye(3)];
robot.Bc=[0.5*robot.delta_t^2*eye(3);
        robot.delta_t*eye(3)];
robot.C=[];
robot.D=[];
robot.Q=[];
robot.R=[];

robot.x(1:robot.nlink*2,1)=[robot.DH(:,1);zeros(robot.nlink,1)];%(theta1,theta2,theta,3,theta1dot,theta2dot,theta3dot)
robot.pos=cap_pos(robot.base,robot.DH,robot.cap);
robot.wx(1:3,1)=robot.pos{robot.nlink}.p(:,2);
robot.wx(4:6,1)=[0;0;0];%endpoint state(x4,y4,z4,x4dot,y4dot,z4dot)
robot.mx=robot.wx;%closest point state

robot.ref.x=robot.x;
robot.innoise=0;
robot.outnoiseself=0;
robot.outnoisestar=0;
robot.obs.xself=[];
robot.obs.xstar=[];
robot.obs.goal=[];

robot.obs.A=robot.A;
robot.obs.B=robot.B;
robot.obs.C=robot.C;
robot.obs.D=robot.D;
robot.obs.Q=robot.Q;
robot.obs.R=robot.R;
robot.score=0;
robot.flag=0;


%For SSA
robot.const.P1=[eye(robot.nlink) zeros(robot.nlink);zeros(robot.nlink) zeros(robot.nlink)];
robot.const.P2=[zeros(robot.nlink) 0.5*eye(robot.nlink);0.5*eye(robot.nlink) zeros(robot.nlink)];
robot.const.P3=[zeros(robot.nlink) zeros(robot.nlink);zeros(robot.nlink) eye(robot.nlink)];

%For collision check
robot.profile={};

%For Optimization
robot.opt.A = [];
robot.opt.b = [];
robot.opt.Aeq = [];
robot.opt.beq = [];
robot.opt.lb = [];
robot.opt.ub = [];

end


