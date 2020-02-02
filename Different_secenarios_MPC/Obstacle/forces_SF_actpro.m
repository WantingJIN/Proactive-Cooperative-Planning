function [ F0,Fe, ang ,fir] = forces_SF_actpro(xh,xr,Con,rob,hum,h_goal)
% Function forces_SF calculate the social force applied on the object based
% on it current state of the state of other agent in the environment
%-----------------------------------------------------------------------------------------------------%
%% Global values
tau = 0.5; 
% A = 300;
% B = 2; %0.8
% A = 100;
% B = 2; %0.8
% A = 200;
% B =2;


A = 200;
B = 2;

% Apro = 200;
% Bpro = 6;

% A=20;
% B=0.08;
% As =250; 
% Bs =10;
% As =250; 
% Bs =10;


As =250; 
Bs =15;
% Aw = 300;
% Bw = 0.1;
Aw = 2000;
Bw = 0.08;

Ao = 400;
Bo = 0.5;
Aso =600; 
Bso =1.2;

k1 = 1.2*10^5;
k2 = 2.4*10^5;

% Environment Parameter
[map_walls, map_obs] = map_def;
[dnum_walls, ~] = size(map_walls);
% Number of walls
num_walls = dnum_walls/2;         
[num_obs, ~] = size(map_obs);


%% Position and velocity decomposition
pos_h = [xh(1) xh(2)];
% only take into consider the difference between the desired velocity and the velocity
%along hum forward direction to calculate the internal force
vel_h = [xh(4)*cos(xh(3)) xh(4)*sin(xh(3))];

pos_r = [xr(1) xr(2)];
vel_r = [Con(1)*cos(xr(3)) Con(1)*sin(xr(3))];

%% Warning area
% Warning area is defined as the intersection of field of view and desired
% direction with range
% simple scenario: only one robot and one human in the corridor

% warn_area = 0;
% R = 8.95; % Take the average value of two medians, or generate random value within the value range
% % d = 0.85;  % Take the average value of two medians
% d =1;
% d_sf = 0.09;
% d_so = 1;
% d_sb = 2;
% e_h = [cos(X_p(3)) sin(X_p(3))];
% e_hT = [-sin(X_p(3)) cos(X_p(3))];
% d_hr = norm(pos_h - pos_r);
% th_hr = (pos_r - pos_h)*e_h'/d_hr;
% dT_hr = sqrt(d_hr^2 - norm((pos_h - pos_r)*e_h')^2);
% 
% if d_hr < R && th_hr > 0 && dT_hr < d   % The robot is inside the warning area of the human
%     % Two kind of behavior, follow or avoid
%     warn_area = 1;
%     if vel_h*vel_r' <= 0    % avoid
%         sub_L = pos_r + d_sf * e_h + d_so * e_hT;
%         sub_R = pos_r + d_sf * e_h - d_so * e_hT;
%         % To choose the appropriate subgoal
%         if norm(pos_h - sub_L) < norm(pos_h - sub_R)
%             h_goal = sub_L;
%         else
%             h_goal = sub_R;
%         end
%     else    % follow
%         sub_B = pos_r - d_sb*e_h;
%         h_goal = sub_B;
%     end
%  
% else
%       h_goal=hum.goal;
% end
%% Social force
fi0=zeros(1,2); % velocity force

fiw1=zeros(1,2);% repulsive
fiw2=zeros(1,2);% compression
fiw3=zeros(1,2);% friction


fio = zeros(1,2);
fir = zeros(1,2);

%% Social force model with explicit collision prediction
e_h = [cos(xh(3)) sin(xh(3))];
e_r = [cos(xr(3)) sin(xr(3))];
rir = hum.r + rob.r;
dir = norm(pos_r - pos_h);
nir = (pos_h - pos_r) / dir;
nir2 = (pos_r - pos_h)/dir;
vir = vel_r - vel_h;

% if e_h * nir2' > sqrt(2)/2
if vir*nir'/norm(vir) >sqrt(2)/2
    fun = @(t)norm(pos_r' + vel_r' * t - pos_h' - vel_h' * t)^2;
    a = norm(vel_r - vel_h)^2;
    b = 2*((pos_r(1) - pos_h(1))*(vel_r(1) - vel_h(1)) + (pos_r(2) - pos_h(2))*(vel_r(2) - vel_h(2)));
    ti = -b / (2*a);
    dti = fun(ti);
%     if ti > 0 && dti<3
   if ti > 0 && dti<9 && a~=0
        pos_rti = pos_r + vel_r * ti;
        pos_hti = pos_h + vel_h * ti;
        dir_ti = norm(pos_hti - pos_rti);
        nir_ti = (pos_hti - pos_rti)/dir_ti;
        fir = As * norm(vel_h)/ti * exp(-sqrt(dir)/Bs) * nir_ti;
    else
         fir = A * exp((rir - dir) / B) * nir;
    end
else
     fir = A * exp((rir - dir) / B) * nir;
end


%% Interaction force in proactive planning
% fir = Apro * exp((rir - dir) / Bpro) * nir;
lamda = 0;
wij = lamda + (1 - lamda)*(1 + e_h* nir2')/2;
fir = wij * fir;



% Initial force towards the goal
% e = (hum.goal-pos_h)/norm(hum.goal-pos_h);
e=(h_goal-pos_h)/norm(h_goal-pos_h);
fi0 = hum.m*(hum.vd*e-vel_h)/tau;

% fie = [0 0];
% Walls forces
for w = 1:num_walls
    xp = pos_h(1);
    yp = pos_h(2);
    rp = [xp yp]';
    ra = map_walls(2*w-1,:)';
    rb = map_walls(2*w,:)';
    xa = ra(1);
    ya = ra(2);
    xb = rb(1);
    yb = rb(2);
    % a point on AB can be parametrized as s(t)=ra+t(rb-ra), t in [0,1]
    % distance from s to p is phi(t)=||s(t)-p||
    % d(phi^2) gives the t which minimizes the distance from p to the
    % line in which AB lives. Since t in [0,1], t_star=min(max(0,t),1);
    % and the distance from p to AB is ||s(t_star)-p||
    t = ((xp - xa)*(xb - xa) + (yp - ya)*(yb - ya)) / (((xb - xa)^2+(yb - ya)^2));
    t_star = min(max(0,t),1);
    rh = ra + t_star*(rb - ra);
    diw = norm(rp - rh);
    niw = (rp - rh)/norm(rp - rh);
    tiw = [-niw(1) niw(2)]';
    fiw1 = fiw1 + Aw * exp((hum.r - diw)/Bw) * niw';
    if diw < hum.r
        fiw2 = fiw2 + k1 * (hum.r - diw) * niw';
        fiw3 = fiw3 - k2 * (hum.r - diw) * (vel_h * tiw) * tiw';
    end
end
fiw = fiw1+fiw2+fiw3;

% Obsctacle force with explicit collision prediction

for o = 1:num_obs
    
    pos_o = map_obs(o,1:2);
    r_o = map_obs(o,3);
    rio = hum.r + r_o;
    dio = norm(pos_o - pos_h);
    nio = (pos_h - pos_o) / dio;
    nio2 = (pos_o - pos_h)/dio;
    lamda = 0;
    wij = lamda + (1 - lamda)*(1 + e_h* nio2')/2;

    if  -vel_h * nio'/norm(vel_h) > sqrt(2)/2
        fun = @(t)norm(pos_o'- pos_h' - vel_h' * t)^2 - (r_o + hum.r)^2;
        a = norm(vel_h)^2;
        b = 2*((pos_o(1) - pos_h(1))*( - vel_h(1)) + (pos_o(2) - pos_h(2))*(- vel_h(2)));
     
        ti = -b / (2*a);
        dti = fun(ti);
        if ti > 0 && dti<1
            pos_hti = pos_h + vel_h * ti;
            dio_ti = norm(pos_hti - pos_o);
            nio_ti = (pos_hti - pos_o)/dio_ti;
            fio = fio + wij * Aso * norm(vel_h)/ti * exp(rio - dio/Bso) * nio_ti;
        else
            fio = fio + wij *Ao * exp((rio - dio) / Bo) * nio;
        end
    else
        fio = fio + wij *Ao * exp((rio - dio) / Bo) * nio;
%         fio = 0;
    end
end


F0 = fi0;
% Fe = fiw1 + fiw2 + fiw3 +wij * fir1;
Fe = fiw + fio + fir;
ang=atan2(e(2),e(1));
end