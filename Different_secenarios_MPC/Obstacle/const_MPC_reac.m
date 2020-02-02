function [g,h] = const_MPC_reac(Con, xr0, PreX_h, rob, hum,TF,t_fine,t_int) % A start, B final ; X intermediate point
% constriant for the reactive planning
% xr0 initial state of the robot in each time iteraction
% xh0 initial state of the human in each time iteraction
%-----------------------------------------------------------------------------------------------------%
%% Parameter

% Environment Parameter
[map_walls,map_obs] = map_def;
[dnum_walls, ~] = size(map_walls);
% Number of walls
num_walls = dnum_walls/2;
[num_obs,~] = size(map_obs);
count =1;

% The initial side of the robot towords the wall
for w = 1:num_walls

    ra = map_walls(2*w-1,:)';
    rb = map_walls(2*w,:)';
    xa = ra(1);
    ya = ra(2);
    xb = rb(1);
    yb = rb(2);
    t = ((xr0(1) - xa)*(xb - xa) + (xr0(2) - ya)*(yb - ya)) / (((xb - xa)^2+(yb - ya)^2));
    t_star = min(max(0,t),1);
    rw = ra + t_star*(rb - ra); % The nearest point on the wall towords the robot
    vec(w,:) = xr0(1:2) - rw';
end

xr=xr0;

for tspan = t_fine : t_fine : TF
    xh = PreX_h(count,:);
    pr = [xr(1) xr(2)]';
    ph = [xh(1) xh(2)]';
    
    % Constriant g1: Distance between the robot and the human
    C = norm(pr - ph);
    g1(count) = hum.r + rob.r + 0.5 - C;
    
%     %Constriant g2: Directional constriant
%     vh = [xh(4)*cos(xh(3)) xh(4)*sin(xh(3))];
%     vr = [Con(count,1)*cos(xr(3)) Con(count,1)*sin(xr(3))];
%     cdir = (vr*(ph - pr) + vh*(pr -ph))/(C^2);
%     g2(count) = cdir - 1;
    
    
    % Constraint g3: distance between the robot and the wall
    for w = 1:num_walls
     
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
        t = ((xr(1) - xa)*(xb - xa) + (xr(2) - ya)*(yb - ya)) / (((xb - xa)^2+(yb - ya)^2));
        t_star = min(max(0,t),1);
        rw = ra + t_star*(rb - ra); % The nearest point on the wall towords the robot
        
        % !!! Should not only calculae the distance, the distance vector
        % towards the wall should be the same with the initial state, the
        % robot should not go cross the wall
        if (vec(w,:))*(pr - rw) > 0
            diw = norm(pr - rw);
        else
            diw = -norm(pr-rw);
        end
        g3((count-1)*num_walls+w) = - diw+ rob.r;
        
    end
    
    % Constriant g4: Time to collision
    % The limit of time to collision is 8 seconds.
    % Calculate the minimum distance between robot and human position in 8
    % seconds, this minimum distance should bigger than the sum of the radius
    
    
%     fun=@(t)norm(pr + vr' * t - ph - vh' * t) ^ 2;
%     
%     ttc=8;
%     a = norm(vr - vh)^2;
%     b = 2 * ((pr(1) - ph(1)) * (vr(1) - vh(1)) + (pr(2) - ph(2)) * (vr(2) - vh(2)));
%     
%     if 0<-b/(2*a) && -b/(2*a)<ttc && a~=0 && b~=0
%         dist = fun(-b/(2*a));
%     else
%         dist=min(fun(0),fun(ttc));
%     end
%     g4(count) = (rob.r + hum.r + 1) - dist;
    
    %Constranit g5: Distance to the obstacle
    for o = 1:num_obs
        pos_obs = map_obs(o,1:2);
        r_obs = map_obs(o,3);
        rio = rob.r + r_obs;
        dio = norm(pos_obs' - pr);
        
        
        g5((count - 1) * num_obs + o) = - dio+ rio+0.2;
        
    end
    
    
    g6(count) = -Con(count,1);
    % Update the robot state
    for tsint = 0 : t_int : t_fine - t_int
        dx_r = system_model_ROB(Con(count,:),xr);
        xr_new = xr' + dx_r * t_int;
        xr = xr_new';
    end
    count = count+1;
    
end
% h= norm(rob.goal-xr(1:2));
h=[];
g=[g1 g3 g5 g6];


end