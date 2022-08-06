% This function implements the jerk-based safe set algorithm (JSSA).
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

function [pos_cmd,replan_feedback,sent_pos,sent_vel,sent_acc,...
          enb_internal_replan,active,jerk_apply] = SafeControl(jerk_ref,nlink,DH_in,RoCap_in,base,margin,nlink_human,HuCap_in,HuCapVel_in,enb,Tc)

persistent m_cur_pos m_status m_inactive_cnt m_host_replan_cnt m_pre_jerk;

if isempty(m_status)
    m_status = 0;
    m_cur_pos = zeros(18, 1);
    m_inactive_cnt = 0;
    m_host_replan_cnt = 0;
    m_pre_jerk=zeros(6,1);
end

pos_cmd = zeros(21,1);
pos_cmd(19:21) = jerk_ref(7:9);
jpos = m_cur_pos(1:6);
jvel = m_cur_pos(7:12);
jacc = m_cur_pos(13:18);

% degree to rad
for i=1:nlink
    jpos(i) = jpos(i)*pi/180;
    jvel(i) = jvel(i)*pi/180;
    jacc(i) = jacc(i)*pi/180;
    jerk_ref(i) = jerk_ref(i)*pi/180;
end

% Set up DH to current robot pose
DH = reshape(DH_in, 4, 10)';
for i=1:nlink
    DH(i,1) = DH(i,1)+jpos(i);
end

% Compute robot pose and human pose
RoCap_full = reshape(RoCap_in, 7, 10)';
RoCapR = RoCap_full(:, 1);
RoCap = RoCap_full(:, 2:7);
robotpos=CapPos(base,DH,RoCap,nlink);

HuCap_full = reshape(HuCap_in, 7, 10)';
HuCapR = HuCap_full(:, 1);
humanpos = zeros(10, 6);
for i=1:nlink_human
    humanpos(i,1:3) = HuCap_full(i, 2:4);% + [obs(1) obs(2) obs(3)];
    humanpos(i,4:6) = HuCap_full(i, 5:7);% + [obs(1) obs(2) obs(3)];
end

HuCapVel_full = reshape(HuCapVel_in, 6, 10)';
humanvel = zeros(10, 6);
for i=1:nlink_human
    humanvel(i,1:3) = HuCapVel_full(i, 1:3);% + [obs(1) obs(2) obs(3)];
    humanvel(i,4:6) = HuCapVel_full(i, 4:6);% + [obs(1) obs(2) obs(3)];
end

% Compute the safety constraint (SSA)
% Critical Points
[dmin,p1,link1,p2,vel2,link2]=CriticlePoints(robotpos,humanpos,humanvel,RoCapR,HuCapR,nlink,nlink_human);
% p1(:, 1:min(5, end))

I = eye(3);
Am = [I,        Tc*I,     0.5*Tc^2*I;
      zeros(3), I,        Tc*I;
      zeros(3), zeros(3), I];
Bm = [1.5*Tc^3*I;
     0.5*Tc^2*I;
     Tc*I];
% Compute Jacobian
nlink = 6;
ncons = 2;  %n_constraint
if size(p1, 2) == 1
    ncons = 1;
end
J = zeros(3, nlink, ncons);
Jd = zeros(3, nlink, ncons);
Jdd = zeros(3, nlink, ncons);
BJ = zeros(9, nlink, ncons);
x_H_predict=zeros(9,ncons);
mx=zeros(9,ncons);
D=zeros(9,ncons);
for i = 1:ncons
    [J(:, :, i),Jd(:, :, i),Jdd(:, :, i)]=jacobiDiff(DH,base,link1(i),p1(:, i),jvel,jacc);
    BJ(:, :, i)=Bm*J(:, :, i);
    % Assume static obs  
    x_H_predict(1:3, i) = p2(1:3, i);
    x_H_predict(4:6, i)=vel2(1:3, i);
    % Robot Cartesian state    
    mx(1:3, i)=p1(:, i);
    mx(4:6, i)=J(:, :, i)*jvel;
    mx(7:9, i)=J(:, :, i)*jacc+Jd(:, :, i)*jvel;

    D(:, i)=Am*mx(:, i) + Bm*(Jdd(:, :, i)*jvel+2*Jd(:, :, i)*jacc) - x_H_predict(:, i);
end
if(~enb || jerk_ref(end))
    jerk_m = jerk_ref(1:6);
else
    jerk_m = jerk_ref(1:6)*0;
end
thres = zeros(ncons, 1);
vet = zeros(ncons, nlink);
phi = zeros(ncons, 1);
for i = 1:ncons
    [thres(i),vet(i, :),phi(i)]=safety_sl(D(:, i),BJ(:, :, i),margin+RoCapR(link1(i))+HuCapR(link2(i)),jerk_m);
end
% modification matrix; can be designed
Q = [1      0      0       0       0       0;
     0      1      0       0       0       0;
     0      0      1       0       0       0;
     0      0      0       1       0       0;
     0      0      0       0       1       0;
     0      0      0       0       0       1];
n_Q = size(Q, 1);
n_S = max(size(thres));
% f = zeros(n_Q, 1);
f = 2 * Q * jerk_m;
% f = 2*reshape(jerk_m, 1, n_Q)*Q;
% x0 = zeros(6, 1);
x0 = [0.1; 0.3; 0.5; 0; 0.2; 0.3];
M = 10000*ones(n_S, 1);
options = optimoptions('quadprog');
options.Display = 'none';  
options.Algorithm = 'active-set';
active = 0;
USE_QP = 1;
jerk_max = 5;   %bound of jerk, for deg
if enb
    if sum(phi>0) > 0
        if ~USE_QP
            vet = vet(1,:);
            thres = thres(1);
            jerk_m=jerk_m-(vet*jerk_m-thres)*Q*vet'/(vet*Q*vet');
        else
%             vet = vet(1,:);   %1:ncons
%             thres = thres(1); %1:ncons
            [jerk_m, ~, flag] = quadprog(Q, f, -vet, -thres, [], [], [], [], x0, options);              
            %flag, jerk_m    %, (jerk_m-jerk1)'
            if (flag == -2)||(modulus(jerk_m) > 100*(jerk_max/180*pi))    %calculate constraint-relaxed opt-solution
                    Q_ext = blkdiag(Q, zeros(n_S));
                    f_ext = [f;M];
                    A_ext = [-vet, -eye(n_S);zeros(n_S, n_Q), -eye(n_S)];
                    b_ext = [reshape(-thres, n_S, 1);zeros(n_S, 1)];
                    x0_ext = zeros(n_Q+n_S, 1);
                    solu_ext = quadprog(Q_ext, f_ext, A_ext, b_ext, [], [], [], [], x0_ext, options);
%                     solu_ext(7:end)';
                    jerk_m(:) = solu_ext(1:6);
            end
        end
%         change = thres-vet*jerk_m;
%         jerk_m=Q*vet'*(change/(vet*Q*vet'));
        pos_cmd(end) = 1;
        m_status = 1;
        replan_feedback = m_host_replan_cnt;%1;
        enb_internal_replan = 0;
        m_inactive_cnt = 0;%m_active_cnt + 1;
        active = 1;
    else
        active = 0;
        if m_inactive_cnt == 5
            enb_internal_replan = 1;
        else
            enb_internal_replan = 0;
        end
        if m_status == 1 %&& m_active_cnt > 1
            m_host_replan_cnt = m_host_replan_cnt+1;
            replan_feedback = m_host_replan_cnt;%0;
            %enb_internal_replan = 1;
        else
            replan_feedback = m_host_replan_cnt;%0;
            %enb_internal_replan = 0;
            %m_active_cnt = 0;
        end
        m_inactive_cnt = m_inactive_cnt + 1;
        m_status = 0;
    end
else
    replan_feedback = 0;
    enb_internal_replan = 0;
    m_status = 0;
end

% rad to deg
jpos = jpos*180/pi;
jvel = jvel*180/pi;
jacc = jacc*180/pi;
jerk_m = jerk_m*180/pi;
if(enb && any(phi>0))
    jerk_abs_max = max(abs(jerk_m));
    if jerk_abs_max > jerk_max
        jerk_m = jerk_m/jerk_abs_max*jerk_max;
    end
end
% for i=1:nlink
%     jpos(i) = jpos(i)*180/pi;
%     jvel(i) = jvel(i)*180/pi;
%     jacc(i) = jacc(i)*180/pi;
%     jerk_m(i) = jerk_m(i)*180/pi;
%     if(enb && phi>0)
%         if jerk_m(i)>jerk_max
%             jerk_m(i) = jerk_max;
%         elseif jerk_m(i)<-jerk_max
%             jerk_m(i) = -jerk_max;
%         end
%     end
% end

% Triple integrator for pos command
Adt = Aj(Tc);
Bdt = Bj(Tc);
pup = [120, 60, 90, 180, 45, 210] - 3;
plo = [-120, -60, -80, -180, -135, -210] + 3;
for i=1:nlink
    x = [jpos(i); jvel(i); jacc(i)];
    unew = Adt*x+Bdt*jerk_m(i);
    if unew(1) > pup(i)
        unew(1) = pup(i);
    elseif unew(1) < plo(i)
        unew(1) = plo(i); 
    end
    pos_cmd(i) = unew(1);
    pos_cmd(i+6) = unew(2);
    pos_cmd(i+12) = unew(3);
    
end

m_cur_pos(1:18) = pos_cmd(1:18);
if pos_cmd(end) == 0
    for i=7:18
        if abs(m_cur_pos(i)) < 0.0001
            m_cur_pos(i) = m_cur_pos(i)*0;
        end
    end
    %m_cur_pos(7:18) = m_cur_pos(7:18)*0;
end

jerk_apply = jerk_m;
m_pre_jerk = jerk_apply;
sent_pos = m_cur_pos(1:6);
sent_vel = m_cur_pos(7:12);
sent_acc = m_cur_pos(13:18);


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
        pt = RoCap(i, (k-1)*3+1:k*3)';
        robotpos(i,(k-1)*3+1:k*3)=(M(1:3,1:3,i+1)*pt+M(1:3,4,i+1))';
    end
end


function [d,p1,link1,p2,vel2,link2]=CriticlePoints(RoCap,HuCap,HuCapVel,rr,hr,nlink,nlink_human)
nr=nlink;
nh=nlink_human;
% dmin=10;
% p1=[0;0;0];
% p2=[0;0;0];
% vel2=[0;0;0];
% link1=1;    %ori 0
% link2=1;    %ori 0
% nD = nh*sum(rr(4:nr)>0);
% Dist_vec = {};
Dist_vec_unit = struct('d', 00, 'link1', 6, 'link2', 4, ...
    'p1', zeros(3, 1), 'p2', zeros(3, 1), 'v2', zeros(3, 1));
Dist_vec = {Dist_vec_unit};    %cell(1, nh*sum(rr(4:nr)>0))
for i = 1:nh*sum(rr(4:nr)>0)
    Dist_vec{end+1} = Dist_vec_unit;
end
% Dist_vec{end}.link1 = 5;
% Dist_vec{:} = {repmat((Dist_vec_unit), 1, nh*sum(rr(4:nr)>0))};
% d = zeros(sum(rr(4:nr)>0), nh);
cnt = 0;
for i=4:nr
    if rr(i)>0  
        for j=1:nh
            [d,~,point1,point2]=DistBetween2Segment(RoCap(i, 1:3),RoCap(i, 4:6),HuCap(j,1:3),HuCap(j,4:6));
            cnt = cnt + 1;
            Dist_vec{cnt}.d=d-rr(i)-hr(j);
            Dist_vec{cnt}.p1 = point1';
            Dist_vec{cnt}.p2 = point2';
            Dist_vec{cnt}.v2=HuCapVel(j,1:3)';
            Dist_vec{cnt}.link1 = i;
            Dist_vec{cnt}.link2 = j;
            if(point2==HuCap(j,4:6))
                Dist_vec{cnt}.v2=HuCapVel(j,4:6)';
            end
        end
    end
end
%sort Dist_vec
nD = size(Dist_vec, 2);
for i = 1:nD
    dmin = 1000;
    minnum = i;
    for j = i:nD
        if Dist_vec{j}.d <dmin
            minnum = j;
            dmin = Dist_vec{j}.d;
        end
    end
    temp = Dist_vec{i};
    Dist_vec{i} = Dist_vec{minnum};
    Dist_vec{minnum} = temp;
end
% if nD >= 5
%     Dist_vec{1}
%     Dist_vec{2}
%     Dist_vec{3}
%     Dist_vec{4}
%     Dist_vec{5}
% end

d = zeros(1, nD);
link1 = zeros(1, nD);
link2 = zeros(1, nD);
p1 = zeros(3, nD);
p2 = zeros(3, nD);
vel2 = zeros(3, nD);
for i = 1:nD
    d(i) = Dist_vec{i}.d;
    link1(i) = Dist_vec{i}.link1;
    link2(i) = Dist_vec{i}.link2;
    p1(:, i) = Dist_vec{i}.p1;
    p2(:, i) = Dist_vec{i}.p2;
    vel2(:, i) = Dist_vec{i}.v2;
end

% if d<dmin
%     dmin=d;
%     p1=point1;
%     p2=point2;
%     vel2=HuCapVel(j,1:3);
%     if(p2==HuCap(j,4:6))
%         vel2=HuCapVel(j,4:6);
%     end
%     link1=i;
%     link2=j;
% end

for i=1:3*nD
    if(vel2(i)>0.1)
        vel2(i) = 0.1;
    elseif vel2(i)<-0.1
        vel2(i) = -0.1;
    end
end


function [J,Jd,Jdd]=jacobiDiff(DH,base,n,p,qd,qdd)
nlink = 6;
p = reshape(p, 3, 1);
z=zeros(3,nlink); % z(:,i) is the z axis of link i in the base coordinate
r=zeros(3,nlink+1); % r_0(:,i) is the coordinate of the i-th origin
omega=zeros(3,nlink); % angular velocity of z
alpha = zeros(3,nlink); % angular acceleration of z
J=zeros(3,nlink); % Jacobian
J_full = zeros(3,nlink,nlink);
Jd=zeros(3,nlink); % Diff(J)
Jd_full=zeros(3,nlink,nlink);
Jdd=zeros(3,nlink); % Diff(Jd)
T=eye(4);
T(1:3, 4) = base;

vr=zeros(3,nlink+1);
ar=zeros(3,nlink+1);

a=DH(:,4);
A=DH(:,3);
D=DH(:,2);
q=DH(:,1);

for i=1:nlink
    z(:,i)=T(1:3,3);
    r(:,i)=T(1:3,4);
    if i>1
        omega(:,i)=omega(:,i-1)+z(:,i).*qd(i);
        alpha(:,i)=alpha(:,i-1)+z(:,i).*qdd(i);
    else
        omega(:,i)=z(:,i).*qd(i);
        alpha(:,i)=z(:,i).*qdd(i);
    end
    T=T * [cos(q(i)) -sin(q(i))*cos(a(i))  sin(q(i))*sin(a(i))  A(i)*cos(q(i));...
           sin(q(i))  cos(q(i))*cos(a(i)) -cos(q(i))*sin(a(i))  A(i)*sin(q(i));...
           0            sin(a(i))                cos(a(i))      D(i);...
           0                0                       0           1];
end
r(:,nlink+1)=T(1:3,4);

for i=1:n
    J(:, i) = [cross(z(:,i),p-r(:, i))];
    for j=1:i
        J_full(:,j, i)=[cross(r(:,j)-r(:,i+1),z(:,j))];
    end
end

vp=J*qd;
vr(:,1)=[0;0;0];
for i=1:nlink
    jacob = J_full(:, :, i);
    vr(:,i+1)=jacob*qd;
end

for i=1:n
    Jd(:,i)=cross(r(:,i)-p, cross(omega(:,i), z(:,i)))+cross(vr(:,i)-vp, z(:,i));
    for j=1:i
        Jd_full(:,j,i)=cross(r(:,j)-r(:,i+1), cross(omega(:,j), z(:,j)))+cross(vr(:,j)-vr(:,i+1), z(:,j));
    end
end

ap=Jd*qd+J*qdd;
ar(:,1)=[0;0;0];
for i=1:nlink
    jacob = J_full(:,:,i);
    jacob_d=Jd_full(:,:,i);
    ar(:,i+1)=jacob_d*qd+jacob*qdd; 
end

for i=1:n
    tmp1 = cross(alpha(:,i),z(:,i))+cross(omega(:,i),cross(omega(:,i),z(:,i)));
    Jdd(:,i)=cross(tmp1, p-r(:,i))...
             +2*cross(cross(omega(:,i),z(:,i)),vp-vr(:,i))+cross(z(:,i), ap-ar(:,i));
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
    
function [thres,vet,phi]=safety_sl(D,BJ,margin,u)
persistent phi_pre;
if isempty(phi_pre)
    phi_pre = 1;
end
P1=[eye(3) zeros(3) zeros(3);
    zeros(3) zeros(3) zeros(3); 
    zeros(3) zeros(3) zeros(3)];
P2=[zeros(3) eye(3) zeros(3);
    zeros(3) zeros(3) zeros(3); 
    zeros(3) zeros(3) zeros(3)];
P3 = [zeros(3) zeros(3) eye(3);
      zeros(3) zeros(3) zeros(3); 
      zeros(3) zeros(3) zeros(3)];
margin=min(margin,0.8);
dm = D+BJ*u;
phi = margin^2*sqrt(dm'*P1*dm)-(sqrt(dm'*P1*dm))^3-3*dm'*P2*dm-dm'*P3*dm;
if phi ~= phi_pre
    phi_pre = phi;
end
d=sqrt(D'*P1*D);
thres=margin^2*d-d^3-3*D'*P2*D-D'*P3*D;
vet=2*D'*P3*BJ+6*D'*P2*BJ;

    
function mtx = Aj(t)
    mtx = [1 t 0.5*t^2;
           0 1 t;
           0 0 1];
       
function mtx=Bj(t)
    mtx = [3/2*t^3; 0.5*t^2; t];

function modu = modulus(vec)
    modu = sqrt(sum(vec.*vec));
    