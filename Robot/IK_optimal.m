% This is the function to compute the robot optimal inverse kinematics.
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

function [obj, x, human_robot_d, err_pos, err_ori] = IK_optimal(robot ,...
    x_nominal, HuCap, ...
    Tmax, step_size, uncertainty_offset, ...
    coef_dist, coef_ori_error)

    x_nominal_rad = x_nominal / 180.0 * pi;

    dir = 1;
    [obj1, x1, human_robot_d1, err_pos1, err_ori1] = SearchJointGoal(x_nominal_rad, ...
        robot, HuCap, Tmax, step_size, ...
        coef_dist, coef_ori_error, ...
        uncertainty_offset, ...
        dir);
    
    dir = -1;
    [obj2, x2, human_robot_d2, err_pos2, err_ori2] = SearchJointGoal(x_nominal_rad, ...
        robot, HuCap, Tmax, step_size, ...
        coef_dist, coef_ori_error, ...
        uncertainty_offset, ...
        dir);
    
    if obj1 > obj2
        obj = obj1;
        x = x1;
        human_robot_d = human_robot_d1;
        err_pos = err_pos1;
        err_ori = err_ori1;
    else
        obj = obj2;
        x = x2;
        human_robot_d = human_robot_d2;
        err_pos = err_pos2;
        err_ori = err_ori2;
    end
    
    x = x/pi*180.0;

end

%%

function [obj_max, x_ret, human_robot_d_ret, err_pos_ret, err_ori_ret] = SearchJointGoal(x_nominal, ...
    robot, HuCap, Tmax, step_size, ...
    coef_dist, coef_ori_error, ...
    uncertainty_offset, ...
    dir)

    J_nominal = jacobian(robot, x_nominal);
    Jnull_nominal = null(J_nominal);
    delta_x_nominal = Jnull_nominal(:, 1);
    toolpose_nominal = FK(robot, x_nominal);
    toolpose_pos_nominal = toolpose_nominal(1:3, 4);
    toolpose_ori_nominal = toolpose_nominal(1:3, 1:3);

    x = x_nominal;
%     cnt = 0;
    
    obj_max = -1e5;

    for z = 1:Tmax
    
%         if human_robot_d - uncertainty_offset < dist_thres && cnt < Tmax
        ns = null(jacobian(robot, x));
        delta_x = ns(:, 1);
        if dir * delta_x_nominal'*delta_x < 0
            delta_x = -delta_x;
        end
        x = x + step_size * delta_x;

        % enforce position
        x_prev = x;
        icop_cnt = 0;
        while icop_cnt < 20
            H = eye(length(x));
            f = -x;
            A = []; b = [];
            J = jacobian(robot, x);
            Aeq = J;
            p = FK(robot, x); p_pose = p(1:3, 4);
            beq = toolpose_pos_nominal - p_pose + J * x;
            options = optimset('Display', 'off');
%             x = quadprog(H,f,A,b,Aeq,beq,options);
            x = quadprog(H,f,A,b,Aeq,beq,[],[],[],options);

            adv = x-x_prev;
            icop_cnt = icop_cnt + 1;
            if adv'*adv < 0.0001
                break
            else
                x_prev = x;
            end
        end

%         else
%             break;
%         end
%     
%         cnt = cnt + 1;

        % error
        DH = robot.DH; DH(:, 1) = DH(:, 1) + x;
        robotpos = CapPos(robot.base, DH, robot.cap, robot.nlink);
        [human_robot_d, i, j] = HumanRobotDistance(robotpos, robot, HuCap);
        
%         human_robot_d
        
        toolpose = FK(robot, x);
        toolpose_pos = toolpose(1:3, 4);
        err_pos = toolpose_pos-toolpose_pos_nominal;
        toolpose_ori = toolpose(1:3, 1:3);
        err_ori = toolpose_ori_nominal / toolpose_ori;
        err_ori_angleax = rotm2axang(err_ori);
        
%         disp(cnt);
%         disp([human_robot_d, i, j]);
%         disp(toolpose_pos_nominal');
%         disp(toolpose_pos');
%         disp(err'*err);

        % objective
        obj = coef_dist * (human_robot_d - uncertainty_offset) - coef_ori_error * err_ori_angleax(4); 
     
        if obj > obj_max
            obj_max = obj;
            x_ret = x;
            for joint_i = 1:robot.nlink
                while x_ret(joint_i) > pi
                    x_ret(joint_i) = x_ret(joint_i) - 2*pi;
                end
                
                while x_ret(joint_i) < -pi
                    x_ret(joint_i) = x_ret(joint_i) + 2*pi;
                end
            end
            human_robot_d_ret = human_robot_d;
            err_pos_ret = err_pos;
            err_ori_ret = err_ori;
        end

    end

end

function [d, imin, jmin] = HumanRobotDistance(robotpos, robot, HuCap)

    mind = 100000;
    for i = 1:robot.nlink
        rr = robot.cap{i}.r;
        if rr <= 1e-5
            continue;
        end
        for j = 1:size(HuCap, 2)
            [distance,~,~,~] = DistBetween2Segment(...
                robotpos(i, 1:3)', robotpos(i, 4:6)',...
                HuCap{j}.p(:, 1), HuCap{j}.p(:, 2));
            distance = distance - rr - HuCap{j}.r;
            if distance < mind
                imin = i; jmin = j;
                mind = distance;
            end
        end
    end
    
    d = mind;

end

function robotpos=CapPos(base,DH,RoCap,nlink)
    robotpos=zeros(10,6);
    M=zeros(4, 4, 11); 
    M(:, :, 1)=[eye(3) base; 
                zeros(1,3) 1];
    for i=1:nlink
        R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
            sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
            0  sin(DH(i,4)) cos(DH(i,4))];
        T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
        M(:, :, i+1)=M(:,:,i)*[R T; zeros(1,3) 1];
        for k=1:2
    %         pt = RoCap(i, (k-1)*3+1:k*3)';
            pt = RoCap{i}.p(:, k);
            robotpos(i,(k-1)*3+1:k*3)=(M(1:3,1:3,i+1)*pt+M(1:3,4,i+1))';
        end
    end
end

function [distance,vec,p1,p2] = DistBetween2Segment(p1, p2, p3, p4)

    u = p1 - p2;
    v = p3 - p4;
    w = p2 - p4;
    
    a = dot(u,u);
    b = dot(u,v);
    c = dot(v,v);
    d = dot(u,w);
    e = dot(v,w);
    D = a*c - b*b;
    sD = D;
    tD = D;
    
    SMALL_NUM = 0.00000001;
    
    % compute the line parameters of the two closest points
    if (D < SMALL_NUM)  % the lines are almost parallel
        sN = 0.0;       % force using point P0 on segment S1
        sD = 1.0;       % to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    else                % get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0)   % sc < 0 => the s=0 edge is visible       
            sN = 0.0;
            tN = e;
            tD = c;
        elseif (sN > sD)% sc > 1 => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        end
    end
    
    if (tN < 0.0)            % tc < 0 => the t=0 edge is visible
        tN = 0.0;
        % recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        elseif (-d > a)
            sN = sD;
        else
            sN = -d;
            sD = a;
        end
    elseif (tN > tD)       % tc > 1 => the t=1 edge is visible
        tN = tD;
        % recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        elseif ((-d + b) > a)
            sN = sD;
        else 
            sN = (-d + b);
            sD = a;
        end
    end
    
    % finally do the division to get sc and tc
    if(abs(sN) < SMALL_NUM)
        sc = 0.0;
    else
        sc = sN / sD;
    end
    
    if(abs(tN) < SMALL_NUM)
        tc = 0.0;
    else
        tc = tN / tD;
    end
    
    % get the difference of the two closest points
    dP = w + (sc * u) - (tc * v);  % = S1(sc) - S2(tc)

    distance = norm(dP);
    outV = dP;
    
    vec = outV;      % vector connecting the closest points
    p1 = p2+sc*u;   % Closest point on object 1 
    p2 = p4+tc*v;   % Closest point on object 2
end