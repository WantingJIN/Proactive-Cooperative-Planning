function [f,f1,f2,f3,f4,f5,f6,w1,w2,w3,w4,w5,w6] = obj_MPC_pro(Con,xr0,xg0,rob,grp,TF,t_fine,t_int) % A start, B final ; X intermediate point
% Objctive function of the optimization problem for one horizon window
% f1: Reach the goal
% f2: Energy consuming
% f3: Human confort
%------------------------------------------------------------------------------------------------%
%% Cost function
f1 = 0;
f2 = 0;
f3 = 0;
f4 = 0;
f5 = 0;
f6 = 0;
f=0;
count = 1;

xr = xr0;
xg = xg0;
for tspan = t_fine : t_fine : TF
    
    % Position and velocity decomposition
    pos_g = [xg(:,1) xg(:,2)];
    % only take into consider the difference between the desired velocity and the velocity
    %along hum forward direction to calculate the internal force
    vel_g = [xg(:,4).*cos(xg(:,3)) xg(:,4).*sin(xg(:,3))];
    
    pos_r = [xr(1) xr(2)];
    vel_r = [Con(1)*cos(xr(3)) Con(1)*sin(xr(3))];
    
    % Update the robot state
    for tsint = 0 : t_int : t_fine - t_int
        dx_r = system_model_ROB(Con(count,:),xr);
        xr_new = xr' + dx_r * t_int;
        [dx_g,fir] = system_model_GRPpre_pro(xg,xr,Con(count,:),rob,grp);
        xg_new = xg + dx_g * t_int;
        xr = xr_new';
        xg = xg_new;
    end
    % Cost function f1: the sum of the distance between the robot current
    % position to the goal position
    % The weight parameter of f1 depends on time, in th beginning the
    % distance to reach the goal is not so important, in the end, the robot
    % should manage to reach the goal
    
    f1 = f1 + norm(rob.goal - xr(1:2)) ^ 2;
    
    
    % Cost function f2: the sum of the control input
    f2 = f2 + norm(Con(count,:)) ^ 2;
    
    
    for i=1:grp.N
        pr = [xr(1) xr(2)]';
        pg = [xg(i,1) xg(i,2)]';
        vg = [xg(i,4)*cos(xg(i,3)) xg(i,4)*sin(xg(i,3))];
        vr = [Con(count,1)*cos(xr(3)) Con(count,1)*sin(xr(3))];
        
        
        % Cost function f3: the interaction force of the human generated by the
        % robot
        %     f3 = f3 +norm(fir)+ norm(fiw);
        f3 = f3 + norm(fir(i,:));
        %     f3 = f3 + norm(fiw);
        %    f3 = f3 + norm(hum.goal-xh(1:2)) ^2;
        
        % Penalty cost function: f4 Direction constriant
        C = norm(pr - pg);
        cdir = (vr*(pg - pr) + vg*(pr -pg))/(C^2);
        if cdir >1
            f4 = f4 + (cdir - 1)*20;
        else
            f4 = f4;
        end
        
        % Penalty cost function: f5 Time to collision constriant
        fun=@(t)norm(pr + vr' * t - pg - vg' * t)^2;
        
        ttc=8;
        a = norm(vr - vg)^2;
        b = 2 * ((pr(1) - pg(1)) * (vr(1) - vg(1)) + (pr(2) - pg(2)) * (vr(2) - vg(2)));
        
        if 0<-b/(2*a) && -b/(2*a)<ttc
            dist = fun(-b/(2*a));
        else
            dist=min(fun(0),fun(ttc));
        end
        
        if dist < rob.r + grp.r(i) + 0.45
            f5 = f5 + ( (rob.r + grp.r(i) + 0.45) - dist)*20;
        else
            f5 = f5;
        end
    end
    
    
    % The potential energy to separate the group f6
    for i = 1 : grp.N
        for j = 1 : grp.N
            if j ~= i
                ei = [cos(xg(i,3)) sin(xg(i,3))];
                ej = [cos(xg(j,3)) sin(xg(j,3))];
                vi = vel_g(i,:);
                vj = vel_g(j,:);
                pi = pos_g(i,:);
                pj = pos_g(j,:);
                % distance between the group member
                diff_d = norm(pi - pj);
                % difference of the heading angle between two members in a group
                diff_e = ei * ej';
                % difference of the magnitude of the velocity of the two member
                diff_v = abs(norm(vi) - norm(vj));
                % when the distance between two members is bigger than a
                % certain value then we no longer take them as a group
                if diff_d < grp.ld
                    % stiffness of the virtual spring between two members in
                    % the group, it depend on the relationship parameter kai,
                    % difference between the heading angle and mangnitude
                    k = grp.kai * (1 + diff_e) / (2 * exp(diff_v));
                    % if diff_d>grp.l0, the spring foece should along
                    % direction i to j, otherwise along j to i
                    f6 = f6 + k * (grp.l0 - diff_d)^2;
                end
            end
        end
    end
    
    
    
    
    count = count + 1;
end
%   f1=  f1 + norm(rob.goal - xr(1:2)) ^ 2;

% f = f2;


w1 = 1;
w2 = 1;
w3 = 0;
w4 = 5;
w5 = 5;
w6 = 1;


f1 = w1*f1;
f2 = w2*f2;
f3 = w3*f3;
f4 = w4*f4;
f5 = w5*f5;
f6 = w6*f6;

w4 = w4 * 20;
w5 = w5 * 20;

f = f1 + f2 + f3 + f4 + f5 + f6;
end