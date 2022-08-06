% This file includes the simulink realtime communication functions.
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

classdef SLRTComm < handle
    % Communication interface between simulink real-time controller and
    % high level control (either in MATLAB or Python, using zmq
    % communication)
    
    properties
        %tg;
        %zmq;
        timeout=60;
        SSA_replan_cnt=0;
        controller_status=0;
        ip = '127.0.0.1';   %local host
    end
    
    methods
        function obj = SLRTComm()
          
        end
        
        % start stream motion (STMO), port 8001
        function startSTMO(obj)
            judp('send',8001,obj.ip,int8(1)); % start stream motion
        end
        
        % stop stream motion (STMO), port 8001
        function endSTMO(obj)
            judp('send',8001,obj.ip,int8(0)); % finish stream motion
            %obj.tg.stop;
        end
        
        % Port 8003, 8004, 8005
        % setup Vm, Am, Jm in deg unit
        % Not used for now
        function setVmAmJm(obj,Vm,Am,Jm)
            judp('send',8003,obj.ip,typecast(single(Vm),'int8'));
            judp('send',8004,obj.ip,typecast(single(Am),'int8'));
            judp('send',8005,obj.ip,typecast(single(Jm),'int8'));
        end
        
        % Port: 25001
        % get robot data
        % q_cur: joint position
        % q_vel: joint velocity
        % replan_request: 1: SSA ongoing. 0: SSA
        % inactive.
        % status: 0: controller idle. 1: controller executing.
        function [q_cur, q_vel, SSA_replan, controller_replan] = getRobData(obj)   %, msg
            judp('send',8106,obj.ip,int8(0));
            judp('send',8106,obj.ip,int8(1));
      
            msg=judp('receive',25001,56,obj.timeout*1000);
            msg=typecast(msg,'single');
            msg=double(msg);

            q_cur=msg(1:6);
            q_vel=msg(7:12);
            ssa=msg(13);
            [ssa obj.SSA_replan_cnt];
            if(obj.SSA_replan_cnt ~= ssa)
                SSA_replan=1;
            else
                SSA_replan=0;
            end
            obj.SSA_replan_cnt = ssa;
            status=msg(14);
            if(obj.controller_status && ~status)
                controller_replan=1;
            else
                controller_replan=0;
            end
            obj.controller_status = status;
        end

        % Port: 8105
        function enableSSA(obj,enb)
            judp('send',8105,obj.ip,typecast(single([enb]),'int8'));
        end
            
        % Port: 8107
        function setRobotProperty(obj,nlink,DH,RoCap,base,margin)
            DH_s = zeros(10, 4);
            RoCap_s = zeros(1, 70);
            DH_s(1:size(DH, 1), :) = DH;
            DH_s = reshape(DH_s.', 1,[]);
            assert(nlink <= 10);
            assert(size(RoCap, 2) <= 10);
            
            for i=1:size(RoCap, 2)
                cap = RoCap{i};
                RoCap_s(1,(i-1)*7+1) = cap.r;
                RoCap_s(1,(i-1)*7+2:(i-1)*7+4) = cap.p(:, 1);
                RoCap_s(1,(i-1)*7+5:i*7) = cap.p(:, 2);
            end
            judp('send',8107,obj.ip,typecast(single([nlink,DH_s,RoCap_s,base.',margin]),'int8'));
        end
        
        % Port: 8108
        function setHumanProperty(obj,huCap, pre_huCap, vel_t)
            HuCap_s = zeros(1, 70);
            HuCap_vel = zeros(1, 60);
            assert(size(huCap, 2) <= 10);
            nlink = size(huCap, 2);
            for i=1:size(huCap, 2)
                cap = huCap{i};
                HuCap_s(1,(i-1)*7+1) = cap.r;
                HuCap_s(1,(i-1)*7+2:(i-1)*7+4) = cap.p(:, 1)';
                HuCap_s(1,(i-1)*7+5:i*7) = cap.p(:, 2)';
                
                pre_cap = pre_huCap{i};
                HuCap_vel(1,(i-1)*6+1:(i-1)*6+3) = (cap.p(:, 1)'-pre_cap.p(:,1)')/vel_t;
                HuCap_vel(1,(i-1)*6+4:i*6) = (cap.p(:, 2)'-pre_cap.p(:,2)')/vel_t;
                
            end
            judp('send',8108,obj.ip,typecast(single([nlink,HuCap_s,HuCap_vel]),'int8'));
        end
        
        function [status,tskcnt] = drvJntTrajWait(obj,q,traj_hz,resample_hz,ovr,task_seq)
            K1 = 1/traj_hz/0.008/resample_hz;
            if(floor(K1) ~= K1)
                disp("resample_hz not supported!");
                status = 0;
                tskcnt = task_seq;
                return
            end
            % q: Nx6
            num_waypoints = size(q, 1);
            traj_q = zeros(1875, 6);
            traj_q(1:num_waypoints, :) = q(:, :);
            traj_q = reshape(traj_q.', 1,[]);
            
            judp('send',8104,obj.ip,typecast(single([traj_q(:).',num_waypoints,traj_hz,resample_hz,ovr,task_seq]),'int8'));
            
            msg=judp('receive',25000,8,obj.timeout*1000);
            msg=typecast(msg,'single');
            msg=double(msg);
            status = logical(msg(1));
            tskcnt = task_seq;%double(msg(2)); % msg(2) = task count
        end
        
        function status = drvJntTraj(obj,q,traj_hz,resample_hz,ovr,task_seq)
            K1 = 1/traj_hz/0.008/resample_hz;
            if(floor(K1) ~= K1)
                disp("resample_hz not supported!");
                status = 0;
                return
            end
            % q: Nx6
            num_waypoints = size(q, 1);
            traj_q = zeros(1875, 6);
            traj_q(1:num_waypoints, :) = q(:, :);
            traj_q = reshape(traj_q.', 1,[]);
            
            judp('send',8104,obj.ip,typecast(single([traj_q(:).',num_waypoints,traj_hz,resample_hz,ovr,task_seq]),'int8'));
            status = 1;
        end
    end
end

