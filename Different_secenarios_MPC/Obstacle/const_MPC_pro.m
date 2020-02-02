function [g,h] = const_MPC_pro(Con,xr0, xh0,rob,hum,TF,t_fine,t_int) % A start, B final ; X intermediate point
%  Constraint of the optimization problem for one horizon window
% xr0 initial state of the robot in each time iteraction (localization data)
% xh0 initial state of the human in each time iteraction (pedestrian detector data)
%-----------------------------------------------------------------------------------------------------%
%% Parameter
% Environment Parameter
[map_walls, map_obs] = map_def;
[dnum_walls, ~] = size(map_walls);
% Number of walls
num_walls = dnum_walls/2;
[num_obs, ~] = size(map_obs);
%% Constraint

g1=[];
g2=[];
g3=[];
g4=[];
g5=[];
g6=[];
% The initial side of the robot towords the wall

for w = 1:num_walls % Constriant g1: Distance between the robot and the human
    
    ra = map_walls(2*w-1,:)';
    rb = map_walls(2*w,:)';
    
    xa = ra(1);
    ya = ra(2);
    xb = rb(1);
    yb = rb(2);
    t = ((xr0(1) - xa)*(xb - xa) + (xr0(2) - ya)*(yb - ya)) / (((xb - xa)^2+(yb - ya)^2));
    t_star = min(max(0,t),1);
    rh = ra + t_star*(rb - ra); % The nearest point on the wall towords the robot
    vec(w,:) = xr0(1:2) - rh';
end

xr = xr0;
xh = xh0;
count = 1;
for tspan = t_fine : t_fine : TF
    pr = [xr(1) xr(2)]';
    ph = [xh(1) xh(2)]';
    vh = [xh(4)*cos(xh(3)) xh(4)*sin(xh(3))];
    vr = [Con(count,1)*cos(xr(3)) Con(count,1)*sin(xr(3))];
    
    % Constriant g1: Distance between the robot and the human
    
    C = norm(pr - ph);
    g1(count) = hum.r + rob.r + 0.5 -C;

    
%     Constraint g2: distance between the robot and the wall
   % Wanting's idea: calculate the distance between the robot center
        % and the wall
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
        rh = ra + t_star*(rb - ra); % The nearest point on the wall towords the robot
        
        % !!! Should not only calculae the distance, the distance vector
        % towards the wall should be the same with the initial state, the
        % robot should not go cross the wall
        if (vec(w,:))*(pr - rh) > 0
            drw = norm(pr - rh);
        else
            drw = -norm(pr - rh);
        end
        g2((count - 1) * num_walls + w) = - drw+ rob.r;
    end
    
    
    % Paolo's idea: Constraint the value of the robot state
%         g2((count -1) * 2 + 1) = -pr(1) + rob.r;
%         g2((count -1) * 2 +2) =  pr(1) - rob.r -5; 
        
 %     Constraint g3: distance between the human and the wall   
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
        t = ((xh(1) - xa)*(xb - xa) + (xh(2) - ya)*(yb - ya)) / (((xb - xa)^2+(yb - ya)^2));
        t_star = min(max(0,t),1);
        rh = ra + t_star*(rb - ra); % The nearest point on the wall towords the robot
        
        % !!! Should not only calculae the distance, the distance vector
        % towards the wall should be the same with the initial state, the
        % human should not go cross the wall, since the robot and the human
        % start in  the same corridor, so the initial direction of the
        % human towards the wall should be the same as robot
        if (vec(w,:))*(ph - rh) > 0
            diw = norm(ph - rh);
        else
            diw = -norm(ph - rh);
        end
        
        g3((count - 1) * num_walls + w) = - diw+ hum.r + 0.2;
        
    end
        % Paolo's idea: Constraint the value of the human state
%     g3((count -1) * 2 + 1) = -ph(1) + hum.r +0.2;
%     g3((count -1) * 2 + 2) =  ph(1) + hum.r +0.2-5; 
        
    
    

    % Constraint g4: distance between the robot and the obstacles
    for o = 1:num_obs
        pos_obs = map_obs(o,1:2);
        r_obs = map_obs(o,3);
        ror = rob.r + r_obs;
        dor = norm(pos_obs' - pr);
        g4((count - 1) * num_obs + o) = - dor+ ror;
        
    end
    
     % Constraint g5: distance between the human and the obstacles
     for o = 1:num_obs
        pos_obs = map_obs(o,1:2);
        r_obs = map_obs(o,3);
        rio = hum.r + r_obs;
        dio = norm(pos_obs' - ph);
        g5((count - 1) * num_obs + o) = - dio+ rio;
     end
     
     
     
    % Constriant g6: v>=0
    g6(count) = -Con(count,1);
    
     
    % Update the robot state
    for tsint = 0 : t_int : t_fine - t_int
        dx_r = system_model_ROB(Con(count,:),xr);
        xr_new = xr' + dx_r * t_int;
        [dx_h,~] = system_model_HUMpre_pro(xh,xr,Con(count,:),rob,hum);
        xh_new = xh' + dx_h * t_int;
        xr = xr_new';
        xh = xh_new';
    end
    count = count + 1;
end

g = [g1 g2 g3 g4 g5 g6];
h = [];

end