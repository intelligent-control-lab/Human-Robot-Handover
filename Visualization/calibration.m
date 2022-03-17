% This is the function to calibrate the control plot.
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

function [cursor_pos_center,cursor_pos_URcorner]=calibration(p)
calibmark.xy = [0,0];
calibmark.handle = plot(calibmark.xy(1),calibmark.xy(2),'o','linewidth',3,'color','r','markersize',14);
set(calibmark.handle,'XDataSource','calibmark.xy(1)');
set(calibmark.handle,'YDataSource','calibmark.xy(2)');

calibmark.xy = [0; 0];
refreshdata([calibmark.handle],'caller');
drawnow;
pause(3)
cursor_pos_center = get(0,'PointerLocation');

calibmark.xy = p;
refreshdata([calibmark.handle],'caller');
drawnow;
pause(3)
cursor_pos_URcorner = get(0,'PointerLocation');

calibmark.xy = [-100; -100];
refreshdata([calibmark.handle],'caller');