function [ F0,Fe, ang ,fir] = forces_SF_actpro(X_p,X_r,Con,rob,hum,h_goal)
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

% A=20;
% B=0.08;
% As =250; 
% Bs =10;
% As =250; 
% Bs =10;


As =250; 
Bs =6;
% Aw = 300;
% Bw = 0.1;
Aw = 2000;
Bw = 0.08;
k1 = 1.2*10^5;
k2 = 2.4*10^5;

% Environment Parameter
map_walls = map_def;
[dnum_walls, ~] = size(map_walls);
% Number of walls
num_walls = dnum_walls/2;         



%% Position and velocity decomposition
pos_h = [X_p(1) X_p(2)];
% only take into consider the difference between the desired velocity and the velocity
%along hum forward direction to calculate the internal force
vel_h = [X_p(4)*cos(X_p(3))-X_p(5)*sin(X_p(3)) X_p(4)*sin(X_p(3))+X_p(5)*cos(X_p(3))];

pos_r = [X_r(1) X_r(2)];
vel_r = [Con(1)*cos(X_r(3)) Con(1)*sin(X_r(3))];

%% Social force model with explicit collision prediction
e_h = [cos(X_p(3)) sin(X_p(3))];
e_r = [cos(X_r(3)) sin(X_r(3))];
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
   if ti > 0 && dti<9
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
% fir = A * exp((rir - dir) / B) * nir;
lamda = 0;
wij = lamda + (1 - lamda)*(1 + e_h* nir2')/2;
fir = wij * fir;


%% Social force
fi0=zeros(1,2); % velocity force
% Interindividual forces
% Obstacles
fiw1=zeros(1,2);% repulsive
fiw2=zeros(1,2);% compression
fiw3=zeros(1,2);% friction
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
%     if diw < hum.r
%         fiw2 = fiw2 + k1 * (hum.r - diw) * niw';
%         fiw3 = fiw3 - k2 * (hum.r - diw) * (vel_h * tiw) * tiw';
%     end
end
fiw = fiw1+fiw2+fiw3;


F0 = fi0;
% Fe = fiw1 + fiw2 + fiw3 +wij * fir1;
Fe = fiw+ fir;

ang=atan2(e(2),e(1));
end