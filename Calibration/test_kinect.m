% This is the program for capturing images from Kinect.
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

clear all;
clc;
close all;

%%%%%%%%%%%%%%%%
% Start Kinect
%%%%%%%%%%%%%%%%
colorVid = videoinput('kinect',1);
framesPerTrig = 6;
set(colorVid,'TriggerRepeat',Inf);%TriggerRepeat
set(colorVid,'FramesPerTrigger',framesPerTrig);%FramesPerTrigger
set(colorVid,'FrameGrabInterval',1);%FrameGrabInterval
start([colorVid]);

pause(1);
for i=1:3
    flushdata(colorVid);
    colorImg = getdata(colorVid);
    lastColorImage = colorImg(:, :, :, 1);
    
    % Marker colors for up to 6 bodies.
    colors = ['r';'g';'b';'c';'y';'m'];
    lastColorImage = flipdim(lastColorImage,2);
    % Display the RGB image.
    imshow(lastColorImage);
    imwrite(lastColorImage, 'calib.jpg');
    continue
end
stop(colorVid);
disp("Done!");


