function [g,h] = const_MPC_pro(Con,xr0, xg0,rob,grp,TF,t_fine,t_int) % A start, B final ; X intermediate point
%  Constraint of the optimization problem for one horizon window
% xr0 initial state of the robot in each time iteraction (localization data)
% xh0 initial state of the human in each time iteraction (pedestrian detector data)
%-----------------------------------------------------------------------------------------------------%
%% Parameter
% Simulation time
% TF =6;
% t_fine = 1/10; % time step for calculating the optimal control input
% t_int = 1/50; % time step for integral

% Environment Parameter
[map_walls, map_obs] = map_def;
[dnum_walls, ~] = size(map_walls);
% Number of walls
num_walls = dnum_walls/2;
[num_obs, ~] = size(map_obs);
%% Constraint
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
    rh = ra + t_star*(rb - ra); % The nearest point on the wall towords the robot
    vec(w,:) = xr0(1:2) - rh';
end

xr = xr0;
xg = xg0;
count = 1;
for tspan = t_fine : t_fine : TF
    % Update the robot state
    for tsint = 0 : t_int : t_fine - t_int
        dx_r = system_model_ROB(Con(count,:),xr);
        xr_new = xr' + dx_r * t_int;
        [dx_g,~] = system_model_GRPpre_pro(xg,xr,Con(count,:),rob,grp);
        xg_new = xg + dx_g * t_int;
        xr = xr_new';
        xg = xg_new;
    end
    
    for i = 1 : grp.N
        pr = [xr(1) xr(2)]';
        pg = [xg(i,1) xg(i,2)]';
        vg = [xg(i,4)*cos(xg(i,3)) xg(i,4)*sin(xg(i,3))];
        vr = [Con(count,1)*cos(xr(3)) Con(count,1)*sin(xr(3))];
        
        % Constriant g1: Distance between the robot and the human
        C = norm(pr - pg);
        g1{i}(count)= grp.r(i) + rob.r + 0.45 -C;
        

        
        % Constraint g3: distance between the human and the wall
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
            t = ((pg(1) - xa)*(xb - xa) + (pg(2) - ya)*(yb - ya)) / (((xb - xa)^2+(yb - ya)^2));
            t_star = min(max(0,t),1);
            rh = ra + t_star*(rb - ra); % The nearest point on the wall towords the robot
            
            % !!! Should not only calculae the distance, the distance vector
            % towards the wall should be the same with the initial state, the
            % robot should not go cross the wall
            if (vec(w,:))*(pg - rh) > 0
                diw = norm(pg - rh);
            else
                diw = -norm(pg - rh);
            end
            
            g3{i}((count - 1) * num_walls + w) = - diw+ grp.r(i) + 0.2;
            
        end
    end
    
    % Constraint g2: distance between the robot and the wall
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
            diw = norm(pr - rh);
        else
            diw = -norm(pr - rh);
        end
        g2((count - 1) * num_walls + w) = - diw+ rob.r + 0.2;
        
    end
   
    
    % Constraint g4: v>=0
    g4(count) = -Con(count,1);
    
    count = count + 1;
end

% h= norm(rob.goal - xr(1:2));
h=[];
g= [g1{1:grp.N} g2 g3{1:grp.N} g4];


end