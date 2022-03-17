% This file builds JPC.
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

function make(type)
    addpath(genpath('sub_models'));
    exp_name = 'icl_control';
    sim_name = [exp_name, '_sim'];
    
    switch type
        case 'sim'
            rtwbuild(sim_name);
        case 'exp'
            rtwbuild(exp_name);
        case 'all'
            rtwbuild(sim_name);
            rtwbuild(exp_name);
        case 'clean'
            if exist('slprj','dir')
                rmdir('slprj','s');
            end
            if exist([sim_name,'_slrealtime_rtw'],'dir')
                rmdir([sim_name,'_slrealtime_rtw'],'s');
            end
            if exist([exp_name,'_slrealtime_rtw'],'dir')
                rmdir([exp_name,'_slrealtime_rtw'],'s');
            end
            delete(['*','.mldatx'],['*','.slxc']);
        otherwise
            disp('Command not supported.');
            return;
    end
end