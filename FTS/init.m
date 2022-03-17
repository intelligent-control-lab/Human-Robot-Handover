% This is the program to initialize force-torque sensor.
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

function init(ser,struct)
    packet = [uint8(struct.pack('>B',uint8(9))),...
            uint8(struct.pack('>B',uint8(hex2dec('10')))),...
            uint8(struct.pack('>H',uint16(hex2dec('019a')))),...
            uint8(struct.pack('>H',uint16(1))),...
            uint8(struct.pack('>B',uint8(2))),...
            uint8(struct.pack('>H',uint16(hex2dec('0200')))),...
            uint8(struct.pack('>H',uint16(hex2dec('cdca'))))];
    ser.write(packet);
end

