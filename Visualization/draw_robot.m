% This function draws the robot capsules.
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

if ~bootup
    for i=1:size(robot.handle, 1)
        for j=1:size(robot.handle, 2)
            delete(robot.handle(i,j));
        end
    end
end

DH = robot.DH;
for i=1:robot.nlink
    DH(i, 1) = robot.DH(i,1) + jpos(i)/180*pi;
end
[~, M] = cap_pos(robot.base, DH, robot.cap);
handle = [];
for i=2:size(M,2)
    rot = M{i}(1:3,1:3);
    trans = M{i}(1:3,4);
    p = robot.cap{i-1}.p;
    c1 = (rot*p(:, 1)+trans)';
    c2 = (rot*p(:, 2)+trans)';
    r = robot.cap{i-1}.r;
    [x1, y1, z1] = cylinder(r, 10, c1, c2);
    handle(i-1, 1) = surf(x1, y1, z1, 'FaceColor',[1, 0, 0],'EdgeColor','None');
    handle(i-1, 2) = surf(sx*r+c1(1), sy*r+c1(2), sz*r+c1(3), 'FaceColor',[1, 0, 0],'EdgeColor','None');
    handle(i-1, 3) = surf(sx*r+c2(1), sy*r+c2(2), sz*r+c2(3), 'FaceColor',[1, 0, 0],'EdgeColor','None');
    alpha(handle(i-1, 1),0.9);
    alpha(handle(i-1, 2),0.9);
    alpha(handle(i-1, 3),0.9);
end
robot.handle = handle;
