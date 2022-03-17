% This function initializes the digital twin simulation.
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

function fighandle=initialize_figure_interact(i,varargin)
fighandle=[0,0,0];
fighandle(1)=figure(i);clf;
set(gcf,'Position',get(0,'ScreenSize'),'color','w')
set(gcf,'renderer','opengl')

if length(varargin)>1
    xlim=varargin{1};
    ylim=varargin{2};
    zlim=varargin{3};
else
    xlim=[-1,3];
    ylim=[-1,1];
    zlim=[0,2];
end

if length(varargin)>4
    zoom(varargin{5});
else
    zoom(1);
end

if length(varargin)>5
    control_plot = varargin{6};
end

if control_plot 
    fighandle(2)=subplot(1,2,1);
end
grid on;
hold on;

axis equal
axis([xlim,ylim,zlim])
lighting=camlight('right');

load('shortbench');
stage.v(:, 3) = stage.v(:, 3)+0.8;
stage.v(:, 1) = stage.v(:, 1)-0.5;
% Rotate bench
switch_v = stage.v(:,1);
stage.v(:,1) = stage.v(:,2);
stage.v(:,2) = switch_v;
stage.v(:,1) = stage.v(:,1) + 0.5; 
stage.color = [1,1,0.5];
patch('Faces',stage.f,'Vertices',stage.v,'FaceColor',stage.color,'EdgeColor','None');

%lighting phong
set(gca,'Color',[0.8 0.8 0.8]);

wall{1}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(1),ylim(2),ylim(2),ylim(1)],[zlim(1),zlim(1),zlim(1),zlim(1)],[0.1,0.1,0.1]);
wall{2}.handle=fill3([xlim(1),xlim(1),xlim(1),xlim(1)],[ylim(1),ylim(1),ylim(2),ylim(2)],[zlim(1),zlim(2),zlim(2),zlim(1)],[0.9,0.9,0.9]);
wall{3}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(2),ylim(2),ylim(2),ylim(2)],[zlim(1),zlim(2),zlim(2),zlim(1)],[0.9,0.9,0.9]);

if length(varargin)>3
    view(varargin{4});
else
    view([1,-0.5,0.4]);
end

%% initialize the control plot
if control_plot
    fighandle(3) = subplot(1,2,2);
    hold on;
    grid on;
    axis equal
    axis([xlim ylim]);
    fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(1),ylim(2),ylim(2),ylim(1)],[zlim(1),zlim(1),zlim(1),zlim(1)],[0.1,0.1,0.1]);

    plot([-1 1],[0 0],':b','linewidth',4)
    plot([0 0],[-1 1],':b','linewidth',4)
end

end