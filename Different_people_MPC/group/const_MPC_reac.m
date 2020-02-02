function [g,h] = const_MPC_reac(Con, xr0, PreS_g, rob, grp,TF,t_fine,t_int) % A start, B final ; X intermediate point
% constriant for the reactive planning
% xr0 initial state of the robot in each time iteraction
% xh0 initial state of the human in each time iteraction
%-----------------------------------------------------------------------------------------------------%
%% Parameter
% Simulation time
% TF =6;    % Horizon time
% t_fine = 1/10; % time step for calculating the optimal control input
% t_int = 1/50; % time step for integral

% Environment Parameter
[map_walls,map_obs] = map_def;
[dnum_walls, ~] = size(map_walls);
% Number of walls
num_walls = dnum_walls/2;
[num_obs,~] = size(map_obs);


g1=[];
g2=[];
g3=[];
g4=[];
% The initial side of the robot towords the wall
for w = 1:num_walls
%     ra = max([map_walls(2*w-1,1) map_walls(2*w,1)]',[map_walls(2*w-1,2) map_walls(2*w,2)]');
%     rb = min([map_walls(2*w-1,1) map_walls(2*w,1)]',[map_walls(2*w-1,2) map_walls(2*w,2)]');
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
count =1;

for tspan = t_fine : t_fine : TF

%     Constraint for group
    for i = 1 : grp.N
        sg = PreS_g{i}(count,:);
        
        pr = [xr(1) xr(2)]';
        pg = [sg(1) sg(2)]';
        
        
        % Constriant g1: Distance between the robot and the human
        C = norm(pr - pg);
        g1{i}(count) = grp.r(i) + rob.r + 0.45 - C;

    end
    
      % Constraint g3: distance between the robot and the wall
    % !! BY suing this method may cause some problem, when robot could
    % satisfy the constriant in the mid iter, the robot can't come back.
    
    % Try another way
    
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
        t = ((pr(1) - xa)*(xb - xa) + (pr(2) - ya)*(yb - ya)) / (((xb - xa)^2+(yb - ya)^2));
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
        g3((count-1)*num_walls+w) = - diw+ rob.r+0.2;
    end



    % Constraint g3: The robot should not cross the wall
    % Move the line toward the dirction of the initial position for the
    % distance of the radius of the robot
%     for w = 1:num_walls
%         if le_b(w)~=0
%             % Calculate the distance should be move along x axis
%             dx = sqrt(rob.r^2 + (rob.r/le_a(w))^2); 
%         else
%              dx = rob.r;
%         end
%         if val(w) > 0
%                 le_c(w) = le_c(w) - le_a(w)*dx;
%         else
%                 le_c(w) = le_c(w) + le_a(w)*dx;
%         end
%         g3((count-1)*num_walls+w) = -(le_a(w)*xr(1) + le_b(w)*xr(2) + le_c(w))*val(w);
%         
%     end
%     
 
    
    
%     
%     for o = 1:num_obs
%         pos_obs = map_obs(o,1:2);
%         r_obs = map_obs(o,3);
%         rio = rob.r + r_obs;
%         dio = norm(pos_obs' - pr);
%         g6((count - 1) * num_obs + o) = - dio+ rio+0.2;
%     end
    
    % Constarint g5: v>=0
    g5(count) = -Con(count,1);
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
g=[g1{1:2} g3 g5];


end