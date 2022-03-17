% This function generates the human capsules.
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

function HuCap=human_cap(n, init_pos)
HM=cell(1,n);
for i=1:n
    HM{i}=eye(4);
end

HuCap={};
HuCap{1}.p=[0   0;
            0   0;
            1.5 1.5];
HuCap{1}.r=0.2;

HuCap{2}.p=[0   0;
            0   0;
            0.9 1.25];
HuCap{2}.r=0.25;

if n==6
    HuCap{3}.r = 0.1;
    HuCap{3}.p = [0   0;
                  0.2 0.5;
                  1.3 0.8];
    HuCap{4}.r = 0.1;
    HuCap{4}.p = [0    0;
                  -0.2 -0.5;
                  1.3  0.8];
    HuCap{5}.r = 0.1;
    HuCap{5}.p = [0    0;
                  0.15 0.4;
                  0.8  0.1];
    HuCap{6}.r = 0.1;
    HuCap{6}.p = [0     0;
                  -0.15 -0.4;
                  0.8   0.1];
elseif n==10
    for i=1:n
        HuCap{i}.r=0.1;
        HuCap{i}.p=[0 0; 0 0; 0 0];
    end
elseif n>2
    t=[0; 0; pi-0.3; pi-0.3; pi+0.3; pi+0.3; pi-0.2; pi-0.2; pi+0.2; pi+0.2];
    for i=3:2:5
        HuCap{i}.r=0.07;
        HuCap{i}.p=[0 0;0 0;0 0.25];
        theta=t(i);
        R=[1 0 0; 0 cos(theta) -sin(theta);0 sin(theta) cos(theta)];
        T=[0;(i-4)*0.2;1.200];
        HM{i}=[R T;0 0 0 1];
        HuCap{i}.p=R*HuCap{i}.p+[T T];
    end
    %%
    for i=4:2:6
        HuCap{i}.r=0.07;
        HuCap{i}.p=[0 0;0 0;0 0.23];
        theta=t(i);
        R=[1 0 0; 0 cos(theta) -sin(theta);0 sin(theta) cos(theta)];
        T=[0;(i-5)*0.3;0.88];
        HM{i}=[R T;0 0 0 1];
        HuCap{i}.p=R*HuCap{i}.p+[T T];
    end
    
    for i=7:2:9
        HuCap{i}.r=0.1;
        HuCap{i}.p=[0 0;0 0;0 0.3];
        theta=t(i);
        R=[1 0 0; 0 cos(theta) -sin(theta);0 sin(theta) cos(theta)];
        T=[0;(i-8)*0.08;0.65];
        HM{i}=[R T;0 0 0 1];
        HuCap{i}.p=R*HuCap{i}.p+[T T];
    end
    
    for i=8:2:10
        HuCap{i}.r=0.08;
        HuCap{i}.p=[0 0;0 0;0 0.27];
        theta=t(i);
        R=[1 0 0; 0 cos(theta) -sin(theta);0 sin(theta) cos(theta)];
        T=[0;(i-9)*0.16;0.3];
        HM{i}=[R T;0 0 0 1];
        HuCap{i}.p=R*HuCap{i}.p+[T T];
    end
end

for i=1:size(HuCap,2)
    HuCap{i}.p = HuCap{i}.p+[init_pos(1) init_pos(1);init_pos(2) init_pos(2);0 0];
end
end

