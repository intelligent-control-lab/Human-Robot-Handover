% This function draws the human capsules.
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

if MODE == 'HumanINTR'
    if bootup
        set(fighandle(1),'currentaxes',fighandle(2))
        HuCap=human_cap(6, init_pos);
        Hhandle=[];
    end
elseif MODE == 'CamerINTR'
    if bootup
%         set(fighandle(1),'currentaxes',fighandle(2))
        HuCap=human_cap(10, init_pos);
        Hhandle=[];
    end
else
    if bootup
        HuCap={};
        HuCap{1}.p = [0 0;0 0;0 0.7];
        HuCap{2}.p = [-0.14 0.07;0.17 0.38;0.54 0.54];
        HuCap{3}.p = [-0.14 0.07;-0.17 -0.38;0.54 0.54];
        Hhandle=[];
    end
end

for i=1:size(Hhandle, 1)
    for j=1:size(Hhandle, 2)
        delete(Hhandle(i, j));
    end
end
Hhandle=[];
[sx, sy, sz] = sphere(10);
for i=1:size(HuCap,2)
    p = HuCap{i}.p;
    c1 = p(:, 1)';
    c2 = p(:, 2)';
    r = HuCap{i}.r;
    [x1, y1, z1] = cylinder(r, 20, c1, c2);
    Hhandle(i, 1) = surf(x1, y1, z1, 'FaceColor',[1, 0, 0],'EdgeColor','None');
    Hhandle(i, 2) = surf(sx*r+c1(1), sy*r+c1(2), sz*r+c1(3), 'FaceColor',[1, 0, 0],'EdgeColor','None');
    Hhandle(i, 3) = surf(sx*r+c2(1), sy*r+c2(2), sz*r+c2(3), 'FaceColor',[1, 0, 0],'EdgeColor','None');
    alpha(Hhandle(i, 1),0.9);
    alpha(Hhandle(i, 2),0.9);
    alpha(Hhandle(i, 3),0.9);
%     if i==1
%         Hhandle(i)=surf(x1, y1, z1, 'facecolor', [1 0 0]);%plot3(p(1,:),p(2,:),p(3,:),'.-','LineWidth',10,'markersize',200);
%     else
%         Hhandle(i)=plot3(p(1,:),p(2,:),p(3,:),'.-','LineWidth',10,'markersize',50);
%     end
%     set(Hhandle(i),'XDataSource','HuCap{i}.p(1,:)');
%     set(Hhandle(i),'YDataSource','HuCap{i}.p(2,:)');
%     set(Hhandle(i),'ZDataSource','HuCap{i}.p(3,:)');
end





