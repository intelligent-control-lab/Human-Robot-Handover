% This is the program to test force-torque sensor.
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
clear all;
clc;
close all;

serial = py.importlib.import_module('serial'); 
py.importlib.reload(serial);
struct=py.importlib.import_module('struct');
ser=connectFTS(serial);

init(ser,struct);
[~,pos]=readdata(ser,struct);
%set zero
FTS_bias = [pos(1), pos(2), pos(3), pos(4), pos(5), pos(6)];

for cnt=1:100
    [~,pos]=readdata(ser,struct);
    FTS_output = pos - FTS_bias;
    disp(FTS_output);
end
termit(ser,struct);
disconnect(ser);
