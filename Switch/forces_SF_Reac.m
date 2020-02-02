function [ F0,Fe, ang ] = forces_SF_Reac(X,hum,h_goal)
% Function forces_SF calculate the social force applied on the object based
% on it current state of the state of other agent in the environment

%% Global values
%% Global values
tau = 0.5; 

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
pos=[X(1) X(2)];
% only take into consider the difference between the desired velocity and the velocity
%along hum forward direction to calculate the internal force
vel=[X(4)*cos(X(3))-X(5)*sin(X(3)) X(4)*sin(X(3))+X(5)*cos(X(3))];

%% Social force
fi0=zeros(1,2); % velocity force
% Interindividual forces
% Obstacles
fiw1=zeros(1,2);% repulsive
fiw2=zeros(1,2);% compression
fiw3=zeros(1,2);% friction
% Initial force towards the goal
e = (h_goal-pos)/norm(h_goal-pos);
fi0 = hum.m*(hum.vd*e-vel)/tau;
% fie = [0 0];
% Walls forces
for w = 1:num_walls
    xp = pos(1);
    yp = pos(2);
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
        fiw3 = fiw3 - k2 * (hum.r - diw) * (vel * tiw) * tiw';
    end
end
F0 = fi0;
Fe = fiw1+fiw2+fiw3;
ang=atan2(e(2),e(1));
end