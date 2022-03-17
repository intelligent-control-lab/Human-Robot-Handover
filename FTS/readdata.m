% This is the program to get force-torque sensor readings.
% Copyright (C) 2022  
% 
% Authors:
% Ruixuan Liu: ruixuanl@andrew.cmu.edu
% Rui Chen: ruic3@andrew.cmu.edu
% Yifan Sun: yifansu2@andrew.cmu.edu
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

function [sta,pos] = readdata(ser,struct)
    buffersize = uint8(80);
    sta = true;
    s = ser.read(buffersize);
    pos=zeros(1,9);
    for i=length(s)-16+1:-1:1
        if (double(s(i))==hex2dec('20')&&double(s(i+1))==hex2dec('4e'))  
            for j=1:3
                temp=struct.unpack('<h',s(i+2*j:i+2*j+1));
                temp=double(temp{1})/100;
                pos(j)=temp;
                temp=struct.unpack('<h',s(i+6+2*j:i+6+2*j+1));
                temp=double(temp{1})/1000;
                pos(j+3)=temp;
            end
            break;
        end
    end
    pos = pos(1:6);
end

