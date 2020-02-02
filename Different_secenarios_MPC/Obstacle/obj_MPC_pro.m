function [f,f1,f2,f3,f4,f5,w1,w2,w3,w4,w5] = obj_MPC_pro(Con,xr0,xh0,rob,hum,TF,t_fine,t_int) % A start, B final ; X intermediate point
% Object funtion for the initial feasible solution
% x_r  is the state of robot and pedestrian update from the sensor
% information in each time iteration
%-----------------------------------------------------------------------------------------------------%
%% Simulation time
% TF = 6;    % Horizon time
% t_fine = 1/10; % time step for calculating the optimal control input
% t_int = 1/50; % time step for integral

f1 = 0;
f2 = 0;
f3 = 0;
f4 = 0;
f5 = 0;
% f6 = 0;
% f7 = 0;
% f8 = 0;


%% Environment Parameter
[map_walls, map_obs] = map_def;
[dnum_walls, ~] = size(map_walls);
% Number of walls
num_walls = dnum_walls/2;
[num_obs, ~] = size(map_obs);

%% Cost functions
xr = xr0;
xh = xh0;
count = 1;
for tspan = t_fine : t_fine : TF
    pr = [xr(1) xr(2)]';
    ph = [xh(1) xh(2)]';
    vh = [xh(4)*cos(xh(3)) xh(4)*sin(xh(3))];
    vr = [Con(count,1)*cos(xr(3)) Con(count,1)*sin(xr(3))];
    
   % Cost function f1: Distance between the last state of the human in
   % horizon time and the goal
    f1 = f1 + norm(rob.goal - xr(1:2)) ^ 2;
    
    % Cost function f2: sum of the control input
    f2 = f2 + norm(Con(count,:))^2;
    % Penalty cost function: f4 Direction constriant
    C = norm(pr - ph);
    cdir = (vr*(ph - pr) + vh*(pr -ph))/(C^2);
    if cdir > 1
        f4 = f4 + (cdir - 1)*20;
    else
        f4 = f4;
    end
    
    
    % Penalty cost function: f5 Time to collision constriant
    fun=@(t)norm(pr + vr' * t - ph - vh' * t)^2;
    
    ttc=8;
    a = norm(vr - vh)^2;
    b = 2 * ((pr(1) - ph(1)) * (vr(1) - vh(1)) + (pr(2) - ph(2)) * (vr(2) - vh(2)));
    
    if 0<-b/(2*a) && -b/(2*a)<ttc
        dist = fun(-b/(2*a));
    else
        dist=min(fun(0),fun(ttc));
    end
    
    if dist < rob.r + hum.r + 0.45
        f5 = f5 + ( (rob.r + hum.r + 0.45) - dist)*20;
    else
        f5 = f5;
    end
    
%     % Penalty cost function : f6 Distance to the wall
%     for w = 1:num_walls
%         
%         ra = map_walls(2*w-1,:)';
%         rb = map_walls(2*w,:)';
%         xa = ra(1);
%         ya = ra(2);
%         xb = rb(1);
%         yb = rb(2);
%         % a point on AB can be parametrized as s(t)=ra+t(rb-ra), t in [0,1]
%         % distance from s to p is phi(t)=||s(t)-p||
%         % d(phi^2) gives the t which minimizes the distance from p to the
%         % line in which AB lives. Since t in [0,1], t_star=min(max(0,t),1);
%         % and the distance from p to AB is ||s(t_star)-p||
%         t = ((xr(1) - xa)*(xb - xa) + (xr(2) - ya)*(yb - ya)) / (((xb - xa)^2+(yb - ya)^2));
%         t_star = min(max(0,t),1);
%         rh = ra + t_star*(rb - ra); % The nearest point on the wall towords the robot
%         
%         % !!! Should not only calculae the distance, the distance vector
%         % towards the wall should be the same with the initial state, the
%         % robot should not go cross the wall
%          drw = norm(pr - rh);
%         drw = norm(pr(1) - xa);
%         smin = rob.r;
%         smax = smin + 0.2;
%         if drw < smax && drw > smin
%             f6 = f6 + 1000000*(smax - drw)/(drw - smin);
%         end
%     end
%     
%     
%     
%     % Penalty cost function f7: Distance to the obstacle
%     for o = 1:num_obs
%         pos_obs = map_obs(o,1:2);
%         r_obs = map_obs(o,3);
%         ror = rob.r + r_obs;
%         dor = norm(pos_obs' - pr);
%         smin = ror;
%         smax = smin + 0.2;
%         if dor < smax && dor > smin
%             f7 = f7 + 1000*(smax - dor)/(dor - smin);
%         end
%         if dor < smin
%             f7  = f7 + 10000*(smin - dor)/dor;
%         end
%     end
%     
%     % Penalty cost function f8: Proxemic constraint
%     C = norm(pr - ph);
%     smin = rob.r + hum.r +0.45;
%     smax = smin + 0.2;
%     
%     if C > smin && C < smax
%         f8 = f8 + 1000*(smax - C)/(C - smin);
%     end
    
    
    %Update the robot state
    for tsint = 0 : t_int : t_fine - t_int    
        dx_r = system_model_ROB(Con(count,:),xr);
        xr_new = xr' + dx_r * t_int;
        [dx_h,fir] = system_model_HUMpre_pro(xh,xr,Con(count,:),rob,hum);
        xh_new = xh' + dx_h * t_int;
        xr = xr_new';
        xh = xh_new';   
    end
    %   for tsint = 0 : t_int : t_fine - t_int
    %         dx_r = system_model_ROB(Con(count,:),xr);
    %         xr_new = xr' + dx_r * t_int;
    %         xr = xr_new';
    %    end
    count = count+1;
    % Cost function f3: the interaction force of the human generated by the
    % robot
    %     f3 = f3 +norm(fir)+ norm(fiw);
    f3 = f3 + norm(fir);
end




% Reach the goal
w1 =0.5;

% Energy consuming
w2 = 1;

% Human comfort
w3 = 0.05;

% Directional cost
w4 = 10;

% Time to collision cost
w5 = 10;

% % Distance to the wall penalty
% w6 = 1;
% 
% % Distance to the obstacle penalty
% w7 = 2;
% 
% % Proxemic distance penalty
% w8 = 1;

f1 = w1*f1;
f2 = w2*f2;
f3 = w3*f3;
f4 = w4*f4;
f5 = w5*f5;
% f6 = w6*f6;
% f7 = w7*f7;
% f8 = w8*f8;


f = f1 + f2  + f3 + f4 + f5;
end