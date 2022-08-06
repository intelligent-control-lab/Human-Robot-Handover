% This file tests JPC.
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

addpath(genpath('lib'));
%% start experimentor
% tg=slrealtime('fanuc_speedgoat');
% load(tg, 'icl_control');
% start(tg);

comm=SLRTComm;
comm.timeout=15;
Sim.comm=comm;
Sim.robid=0;

comm.startSTMO;
pause(0.5);
comm.setVmAmJm([195 175 180 360 360 550],[640 575 592 1184 1184 1000],[2000 2000 3895 7790 7790 4000]*0.9);

% Larger resample, more accurate tracking
traj = [-2 0 0 0 0 0;
        -4 0 0 0 0 0;
        -8 0 0 0 0 0;
        -10 0 0 0 0 0;
        -12 0 0 0 0 0;
        -14 0 0 0 0 0];
traj_hz = 1;
resample_hz = 5;
ovr = 0.1;
% comm.drvJntTrajWait(traj, traj_hz, resample_hz, ovr, 1);
comm.drvJntTraj(traj, traj_hz, resample_hz, ovr, 1);

pause(1);
resample_hz = 25;
new_traj = [-8 4 0 0 0 0;
            -8 6 0 0 0 0;
            -8 8 0 0 0 0;
            -8 10 0 0 0 0;
            -8 12 0 0 0 0;];
comm.drvJntTraj(new_traj, traj_hz, resample_hz, ovr, 2);
pause(3);

new_traj = [-4 8 0 0 0 0;
            0 4 0 0 0 0];
comm.drvJntTraj(new_traj, traj_hz, resample_hz, ovr, 3);

pause(1);

new_traj = [-3 5 0 0 0 0;
            -2 3 0 0 0 0;
            -1 2 0 0 0 0;
            0 1 0 0 0 0;
            0 0 0 0 0 0];
comm.drvJntTraj(new_traj, traj_hz, resample_hz, ovr, 4);
%drvJntTrajWait

[cur_q, cur_v] = comm.getRobData;

comm.endSTMO;
pause(0.5);
% stop(tg);