% This is the robot handover pipeline.
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

% clear all;
clc;
clf;
close all;
ROBOT = 'LRMate200iD7L';
MODE = 'HumanINTR';%'CamerINTR'; %'CamerINTR'; % HumanINTR for mouse control; HuRobINTR: perception using camera.W
USE_ROBOT = 0; % True: real robot involved. False: only target machine. No robot needed.
enbSSA = 1; % 1: enable SSA. 0: disable SSA.
tasks = load('task.txt')';  
USE_GRIPPER = 0;
USE_FTS = 0;
load_dependencies;  %load relevent folder
num_tasks = size(tasks, 2); %size(tasks, 2)
task_id = 0;
% task_id_last = -1;
bootup = true;
robot = robot_property(ROBOT);

% Setup sound
correct_pos_sound_filename = 'correct_pos_sound.wav';
[correct_pos_sound,correct_pos_sound_Fs] = audioread(correct_pos_sound_filename);
samples = [1,floor(0.3*correct_pos_sound_Fs)];
[correct_pos_sound,correct_pos_sound_Fs] = audioread(correct_pos_sound_filename,samples);

% Setup Gripper
if USE_GRIPPER
    setup_gripper;
end

% Setup FTS
if USE_FTS
    serial = py.importlib.import_module('serial'); 
    py.importlib.reload(serial);
    struct=py.importlib.import_module('struct');
    ser=connectFTS(serial);
    init(ser,struct);
end

switch MODE
    case 'HumanINTR'
        init_pos = [1,1];
        % Intialize figure
        fighandle = initialize_figure_interact(2, [-2, 3], [-2, 2], [0, 3], [1, -1, 2], 1, 1);
        % Calibration
        text1handle = text(0, max(ylim)+1, max(zlim)+0.5, 'Please calibrate...');
        [Center, URCorner] = calibration(init_pos);
        set(fighandle(1), 'currentaxes', fighandle(2))
        set(text1handle, 'string','Test runing...')
    case 'CamerINTR'
        init_pos = [0,0.5];
        %%%%%%%%%%%%%%%%
        % Start Kinect
        %%%%%%%%%%%%%%%%
        depthVid = videoinput('kinect',2);
        prop = getselectedsource(depthVid);
        prop.EnableBodyTracking = 'on';
        framesPerTrig = 6;
        set(depthVid,'TriggerRepeat',Inf);%TriggerRepeat
        set(depthVid,'FramesPerTrigger',framesPerTrig);%FramesPerTrigger
        set(depthVid,'FrameGrabInterval',1);%FrameGrabInterval
        start(depthVid)
        Trans = load("Calibration/camera_transformation.txt");
        fighandle = initialize_figure_interact(2, [-2, 3], [-2, 2], [0, 3], [1, -2, 1], 1, 0);
        text1handle = text(0, max(ylim)+1, max(zlim)+0.5, 'Please calibrate...');
        set(text1handle, 'string','Test runing...')
end
% Draw figures
draw_HR_skeleton;
plot3([0, 1], [1, 0], [0, 0]);
pre_HuCap = HuCap;
vel_t = 1000;

% Setup Target PC
% tg = slrealtime('fanuc_speedgoat');
% if USE_ROBOT
%     load(tg, 'icl_control');
% else
%     load(tg, 'icl_control_sim');
% end
% start(tg);

comm=SLRTComm;
comm.ip = '127.0.0.1';    %127.0.0.1 / 192.168.7.5
comm.timeout=10;
comm.startSTMO;
pause(0.5);
comm.setRobotProperty(robot.nlink,robot.DH,robot.cap,robot.base,robot.ssa_margin);
comm.setHumanProperty(HuCap,pre_HuCap,vel_t);
pause(0.1);
comm.enableSSA(enbSSA);
[jpos, jvel, SSA_status, controller_status] = comm.getRobData;
draw_robot;
pause(0.2);

% CFS Parameters
traj_hz = 1; % Assume traj is 1hz.
resample_hz = 25;
assert(floor((1/traj_hz)/0.008/resample_hz) == ((1/traj_hz)/0.008/resample_hz));
u = init_pos;
human_pre_pos = init_pos;

%% begin
replan_cnt = 0;
bootup = false;
STATE = "init_s"; STATE_last = "none";
goal_type = "regular"; goal_type_last = "none";
goal = zeros(6, 1);
vel_timer = tic;
mouse_sampletimer = vel_timer;
USE_MOUSE = 1;
if USE_MOUSE
    mouse_traj = [0;1;1];
else 
    load('mouse_traj_sample.mat');
end
wrist_trigger_cnt = 0;
home_pos = [0;0;0;0;-90;0];
cnt = 0;
while true
    if STATE ~= STATE_last || goal_type ~= goal_type_last || task_id ~= task_id_last
        disp([STATE goal_type task_id]);
        STATE_last = STATE;
        goal_type_last = goal_type;
        task_id_last = task_id;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update the agent position %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    switch MODE
        case 'HumanINTR'
            if USE_MOUSE
                cursor_pos_current = get(0,'PointerLocation');
                unew = (cursor_pos_current - Center)./(URCorner - Center).*init_pos  % normalized
                mouse_traj(:, end+1) = [toc(mouse_sampletimer);unew'];
            else
                curr_tim = toc(mouse_sampletimer);
                while curr_tim > mouse_traj(1, i)
                    i = i + 1;
                    if i > size(mouse_traj, 2)
                        break
                    end
                end
                if i > size(mouse_traj, 2)
                    break
                else
                    unew = (curr_tim-mouse_traj(1, i-1))/(mouse_traj(1, i)-mouse_traj(1, i-1)) ...
                        *(mouse_traj(2:3, i)-mouse_traj(2:3, i-1)) + mouse_traj(2:3, i-1);
                    unew = unew';
                end
            end
            xref=HuCap{1}.p(1,1); 
            yref=HuCap{1}.p(2,1);
            for i=1:size(HuCap, 2)
                HuCap{i}.p=HuCap{i}.p-[xref xref;yref yref;0 0]+[[unew';0] [unew';0]];
            end
            u = unew;
            human_cur_pos = u;
            deliver_pos = HuCap{4}.p(:,2);
        case 'CamerINTR'
            while true
                flushdata(depthVid);
                [frameDataDepth, timeDataDepth, metaData] = getdata(depthVid);               
                metaDataDepth = metaData(framesPerTrig);
                if  sum(metaDataDepth.IsBodyTracked) > 0
                    trackedBodies = find(metaDataDepth.IsBodyTracked);

                    % Find number of Skeletons tracked.
                    nBodies = length(trackedBodies);

                    % Get the joint indices of the tracked bodies with respect to the color image.
                    colorJointIndices = metaDataDepth.JointPositions(:, :, trackedBodies);

                    % Overlay the skeleton on this RGB frame.
                    x = -colorJointIndices(:,1);
                    y = -colorJointIndices(:,2);
                    z = colorJointIndices(:,3);
                    for i=1:size(colorJointIndices, 1)
                        pt = [x(i); y(i); z(i)];
                        pt = Trans(1:3, 1:3)*pt+Trans(1:3, 4);
                        x(i) = pt(1);
                        y(i) = pt(2);
                        z(i) = pt(3);
                    end

                    HuCap{1}.p=[x(4),x(4);y(4),y(4);z(4),z(4)];       %head-head
                    HuCap{2}.p=[x(3),x(1);y(3),y(1);z(3),z(1)];       %shoulder center-hip center
                    HuCap{5}.p=[x(5),x(6);y(5),y(6);z(5),z(6)];       %shoulder left-elbow left
                    HuCap{6}.p=[x(6),x(7);y(6),y(7);z(6),z(7)];      %elbow left-wrist left
                    HuCap{3}.p=[x(9),x(10);y(9),y(10);z(9),z(10)];    %shoulder right-elbow right
                    HuCap{4}.p=[x(10),x(11);y(10),y(11);z(10),z(11)]; %elbow right-wrist right
                    HuCap{9}.p=[x(13),x(14);y(13),y(14);z(13),z(14)]; %hip left-knee left
                    HuCap{10}.p=[x(14),x(15);y(14),y(15);z(14),z(15)]; %knee left-ankle left
                    HuCap{7}.p=[x(17),x(18);y(17),y(18);z(17),z(18)]; %hip right-knee right
                    HuCap{8}.p=[x(18),x(19);y(18),y(19);z(18),z(19)]; %knee right-ankle right
                    break;
                end
            end
            deliver_pos = HuCap{4}.p(:,2);
    end
    draw_HR_skeleton;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update environment on tg %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    vel_t = toc(vel_timer);
    comm.setHumanProperty(HuCap, pre_HuCap, vel_t);
    pre_HuCap = HuCap;
    vel_timer = tic;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Get current pos and replan feedback %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    [jpos, jvel, ssa_replan_request, controller_replan] = comm.getRobData;
    jpos';   %just to output jpos
    %%%%%%%%%%%%%%%%%
    % State Machine %
    %%%%%%%%%%%%%%%%%
    switch STATE
        case "init_s"
            STATE = "percep";
        case "percep"
            STATE = "ECplan";
            right_wrist = HuCap{4}.p(:,2);

            if task_id == 0 && right_wrist(1)>0.40 && right_wrist(1)<0.70 && ...
               right_wrist(2)>0.20 && right_wrist(2)<0.60
                sound(correct_pos_sound,correct_pos_sound_Fs);
                wrist_trigger_cnt = wrist_trigger_cnt + 1;
                if wrist_trigger_cnt > 1
                    task_id = 1;
                end
            else
                wrist_trigger_cnt = 0;
            end
        case "ECplan"
            % Select task goal 
            enable_plan = 1;
            if(task_id > 0 && task_id <= num_tasks)
                goal = tasks(:, task_id);
                enb_ssa = 0;
                comm.enableSSA(enb_ssa);
                ref_traj = [jpos'+(goal'-jpos')/5;
                            jpos'+(goal'-jpos')/5*2;
                            jpos'+(goal'-jpos')/5*3;
                            jpos'+(goal'-jpos')/5*4;
                            goal'];
                replan_cnt = replan_cnt + 1;
            elseif goal_type == "deliver" 
                goal_cart = deliver_pos + [0.1;-0.2;0.2];% offset
                [goal_nominal, status] = IK(robot, jpos, goal_cart);
                Tmax = 100;
                step_size = 0.1;
                coef_dist = 0.95;
                coef_ori_error = 1-coef_dist;
                
                [obj, goal, human_robot_d, err_pos, err_ori] = IK_optimal(robot, ...
                                                                          goal_nominal, HuCap(1:6), ...
                                                                          Tmax, step_size, 0, ...
                                                                          coef_dist, coef_ori_error);
                enb_ssa = 0;
                comm.enableSSA(enb_ssa);
                ref_traj = [jpos'+(home_pos'-jpos')/2;
                            home_pos';
                            home_pos'+(goal'-home_pos')/2;
                            goal'];
                if ~status % Invalid IK
                    STATE = "percep";
                    continue;
                else % Valid IK
                    replan_cnt = replan_cnt + 1;
                end
            elseif goal_type == "delivering"
                enb_ssa = 0;
                comm.enableSSA(enb_ssa);
                pause(0.1);
            elseif goal_type == "wait_human"
                enb_ssa = 0;
                comm.enableSSA(enb_ssa);
            elseif goal_type == "return"
                task_id = 1;
                goal = tasks(:, task_id);
                enb_ssa = 0;
                comm.enableSSA(enb_ssa);
                ref_traj = [jpos'+(home_pos'-jpos')/2;
                            home_pos';
                            home_pos'+(goal'-home_pos')/2;
                            goal'];
                replan_cnt = replan_cnt + 1;
            elseif goal_type == "return_to_home"
                task_id = 0;
                goal = home_pos;
                enb_ssa = 0;
                comm.enableSSA(enb_ssa);
                ref_traj = [jpos'+(goal'-jpos')/3;
                            jpos'+(goal'-jpos')/3*2;
                            goal'];
                replan_cnt = replan_cnt + 1;
            else
                goal = home_pos;
                if enbSSA
                    enb_ssa = 1;
                else
                    enb_ssa = 0;
                end
                comm.enableSSA(enb_ssa);
                if max(abs(goal-jpos))>0.5
                    ref_traj = [jpos'+(goal'-jpos')/3;
                                jpos'+(goal'-jpos')/3*2;
                                goal'];
                else
                    ref_traj = goal';
                end
                replan_cnt = replan_cnt + 1;
            end
            STATE = "gotask";
        case "gotask"
            if ssa_replan_request % SSA triggered replan
                STATE = "percep";
            elseif(robot_reached_goal(goal, jpos, 0.05)) % Finished task.
                if goal_type == "deliver"
                     goal_type = "delivering";
                     pause(0.1);
                     if USE_FTS
                         FTS_bias = getzdata(ser,struct);
                         disp(["FTS bias ", FTS_bias]);
                     end
                elseif goal_type == "delivering"
                    if USE_FTS && USE_GRIPPER
                        z_force = getzdata(ser,struct)-FTS_bias;
                        disp(["delivering: ", FTS_bias, z_force])
                        
                        if z_force>8
                            gripper.open(gripper_m);
                            goal_type = "wait_human";
                            pause(3);
                        end
                        
                        FTS_bias_wait_human = getzdata(ser,struct);
                        
                        disp(["delivering update wait human bias ", FTS_bias_wait_human]);
                    else
                        goal_type = "wait_human";
                    end
                elseif goal_type == "wait_human"
                    if USE_FTS && USE_GRIPPER
                        z_force = getzdata(ser,struct)-FTS_bias_wait_human;
                        disp(["wait human: ", FTS_bias_wait_human, z_force]);
                        if z_force < -8
                            gripper.close(gripper_m);
                            goal_type = "return";
                            task_id = 1;
                        end
                    else
                        goal_type = "return";
                        task_id = 1;
                    end
                elseif goal_type == "return"
                    if USE_GRIPPER
                        pause(0.5);
                        gripper.open(gripper_m);
                    end
                    goal_type = "return_to_home";
                elseif goal_type == "return_to_home"
                    task_id = 0;
                    goal_type = "regular";
                elseif task_id == num_tasks
                    goal_type = "deliver";
                    if USE_GRIPPER
                        gripper.close(gripper_m);
                        pause(2);
                    end
                end
                pause(0.01);
                STATE = "nextpt";
            else % Unfinished task
                comm.drvJntTraj(ref_traj, traj_hz, resample_hz, 0.1, replan_cnt);
            end
        case "nextpt"
            if(task_id > 100)
                STATE = "brksys";
            else
                STATE = "percep";
                if task_id ~= 0
                    task_id = task_id + 1;
                end
            end
        case "brksys"
            break;
    end
    
    %%%%%%%%%%%%%%%%%
    % visualization %
    %%%%%%%%%%%%%%%%%
    draw_robot;
    drawnow;
    if USE_GRIPPER
        gripper_m = gripper.connect('tcpip', '192.168.7.3', 502);
    end
end

disp("Simulation Done!");
comm.endSTMO;
pause(0.5);
% stop(tg);
if MODE == 'CamerINTR'
    stop(depthVid);
end
if USE_FTS
    termit(ser,struct);
    disconnect(ser);
end
set(text1handle,'string','Test ended')
