function [ F0,Fe, ang ,fir,fiw] = forces_grpSF_Reac(sg, grp,g_goal)
% Function forces_SF calculate the social force applied on the object based
% on it current state of the state of other agent in the environment

%% Global values
tau = 0.5;

A = 200;
B = 0.08; %0.8

As =150;
Bs =10;

Aso =150;
Bso =2;

Aw = 200;
Bw = 0.4;

k1 = 700;
k2 = 1400;

Ao = 100;
Bo = 0.1;



% Environment Parameter
[map_walls,map_obs] = map_def;
[dnum_walls, ~] = size(map_walls);
% Number of walls
num_walls = dnum_walls/2;
[num_obs, ~] = size(map_obs);

%% Position and velocity decomposition
pos_g = [sg(:,1) sg(:,2)];
% only take into consider the difference between the desired velocity and the velocity
%along hum forward direction to calculate the internal force
vel_g = [sg(:,4).*cos(sg(:,3)) sg(:,4).*sin(sg(:,3))];

% pos_r = [sr(1) sr(2)];
% vel_r = [Con(1)*cos(sr(3)) Con(1)*sin(sr(3))];

%% Warning area
% Warning area is defined as the intersection of field of view and desired
% direction with range
% simple scenario: only one robot and one human in the corridor

% warn_area = 0;
% R = 8.95; % Take the average value of two medians, or generate random value within the value range
% % d = 0.85;  % Take the average value of two medians
% d =1;
% d_sf = 0.09;
% d_so = 2;
% d_sb =1.7;
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
%         sub_L = pos_r + d_sf * e_h + d_so   * e_hT;
%         sub_R = pos_r + d_sf * e_h - d_so  * e_hT;
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
%       h_goal=hgoal;
% end

%% Social force model with explicit collision prediction
% e_h = [cos(xg(:,3)) sin(xg(:,3))];
% e_r = [cos(xr(3)) sin(xr(3))];
% rir = grp.r + rob.r;
% dir = norm(pos_r - pos_g);
% nir = (pos_g - pos_r) / dir;
% nir2 = (pos_r - pos_g)/dir;
% if e_h * nir2' > sqrt(2)/2
%     fun = @(t)norm(pos_r' + vel_r' * t - pos_g' - vel_g' * t)^2;
%     a = norm(vel_r - vel_g)^2;
%     b = 2*((pos_r(1) - pos_g(1))*(vel_r(1) - vel_g(1)) + (pos_r(2) - pos_g(2))*(vel_r(2) - vel_g(2)));
%     ti = -b / (2*a);
%     dti = fun(ti);
%     if ti > 0 && dti < 4
%         pos_rti = pos_r + vel_r * ti;
%         pos_hti = pos_g + vel_g * ti;
%         dir_ti = norm(pos_hti - pos_rti);
%         nir_ti = (pos_hti - pos_rti)/dir_ti;
%         fir = As * norm(vel_g)/ti * exp(-sqrt(dir)/Bs) * nir_ti;
%     else
%         fir = A * exp((rir - dir) / B) * nir;
%     end
% else
% %     fir = A * exp((rir - dir) / B) * nir;
%      fir = 0;
% end
% %   fir = A * exp((rir - dir) / B) * nir;
%
% % % The anisotropy effect of the interaction force around the pedestrians
% % % With lamda=0 the robot motion and no effect on human behavior from
% % % backwards and lamda=1 when the robot effect is isotropy from all directions
% lamda = 0;
% wij = lamda + (1 - lamda)*(1 + e_h* nir2')/2;
%  fir = wij * fir;


%% Social force
fi0=zeros(grp.N,2); % velocity force

fiw = zeros(grp.N,2);% interatcion force generated by the wall
fiw1 = zeros(1,2);% repulsive
fiw2 = zeros(1,2);% compression
fiw3 = zeros(1,2);% friction

fig  = zeros(grp.N,2); % group spring force

fio = zeros(grp.N,2);% interaction force generated by the obstacle


fir = zeros(grp.N,2); % interaction force generated by the robot

% Initial force towards the goal
for i = 1:grp.N
    e(i,:) = (g_goal - pos_g(i,:)) / norm(g_goal- pos_g(i,:));
    fi0(i,:) = grp.m(i) * (grp.vd(i) * e(i,:) - vel_g(i,:)) / tau;
end
% fie = [0 0];

% Walls forces
for i = 1:grp.N
    for w = 1:num_walls
        xg = pos_g(i,1);
        yg = pos_g(i,2);
        rg = [xg yg]';
%         ra = max([map_walls(2*w-1,1) map_walls(2*w,1)]',[map_walls(2*w-1,2) map_walls(2*w,2)]');
%         rb = min([map_walls(2*w-1,1) map_walls(2*w,1)]',[map_walls(2*w-1,2) map_walls(2*w,2)]');
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
        t = ((xg - xa)*(xb - xa) + (yg - ya)*(yb - ya)) / (((xb - xa)^2+(yb - ya)^2));
        t_star = min(max(0,t),1);
        rh = ra + t_star * (rb - ra);
        diw = norm(rg - rh);
        niw = (rg - rh)/norm(rg - rh);
        tiw = [-niw(1) niw(2)]';
        fiw1 = fiw1 + Aw * exp((grp.r(i) - diw)/Bw) * niw';
        if diw < grp.r(i)
            fiw2 = fiw2 + k1 * (grp.r(i) - diw) * niw';
            fiw3 = fiw3 - k2 * (grp.r(i) - diw) * (vel_g(i,:) * tiw) * tiw';
        end
    end
    fiw(i,:) = fiw1 + fiw2 + fiw3;
end

% Group spring force
for i = 1 : grp.N
    for j = 1 : grp.N
        if j ~= i
            ei = [cos(sg(i,3)) sin(sg(i,3))];
            ej = [cos(sg(j,3)) sin(sg(j,3))];
            vi = vel_g(i,:);
            vj = vel_g(j,:);
            pi = pos_g(i,:);
            pj = pos_g(j,:);
            nij = (pi - pj)/norm(pi - pj);
            tij = [-nij(1) nij(2)]';
            rij = grp.r(i) + grp.r(j);
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
                fig(i,:) = fig(i,:) + k * (grp.l0 - diff_d) * nij;
                
                %Interaction force to keep personal distance
                fig(i,:) = fig(i,:) + A * exp((rij - diff_d) / B) * nij;
                
%                 if diff_d < rij
%                     fig(i,:) = fig(i,:) + k1 * (rij - diff_d) * nij;
%                     fig(i,:) = fig(i,:) - k2 * (rij - diff_d) * (vel_g(i,:) * tij) * tij';
%                 end
                 % 
            end
        end
    end
end

% % Obstacle force
% for o = 1:num_obs
%     pos_obs = map_obs(o,1:2);
%     r_obs = map_obs(o,3);
%     rio = hum.r + r_obs;
%     dio = norm(pos_obs - pos_h);
%     nio = (pos_h' - pos_obs') / dio;
%     tio = [-nio(1) nio(2)]';
%     fio1 = fio1 + Ao * exp((rio - dio)/Bo) * nio';
% %      if dio < rio
% %         fio2 = fio2 + k1 * (rio - dio) * nio';
% %         fio3 = fio3 - k2 * (rio - dio) * (vel_h * tio) * tio';
% %     end
% end
% fio = fio1 + fio2 + fio3;


% Obsctacle force with explicit collision prediction
for i = 1:grp.N
    e_g = [cos(sg(i,3)) sin(sg(i,3))];
    for o = 1:num_obs
        pos_o = map_obs(o,1:2);
        r_o = map_obs(o,3);
        rio = grp.r(i) + r_o;
        dio = norm(pos_o - pos_g(i,:));
        nio = (pos_g(i,:) - pos_o) / dio;
        nio2 = (pos_o - pos_g(i,:))/dio;
        lamda = 0;
        wij = lamda + (1 - lamda)*(1 + e_g* nio2')/2;

        if -vel_g(i,:) * nio'/norm(vel_g(i,:)) > sqrt(2)/2
            fun = @(t)norm(pos_o'- pos_g(i,:)' - vel_g(i,:)' * t)^2 - (r_o + grp.r(i))^2;
            a = norm(vel_g(i,:))^2;
            b = 2*((pos_o(1) - pos_g(i,1))*( - vel_g(i,1)) + (pos_o(2) - pos_g(i,2))*(- vel_g(i,2)));
            
            ti = -b / (2*a);
            dti = fun(ti);
            if ti > 0 && dti < 1
                pos_gti = pos_g(i,:) + vel_g(i,:) * ti;
                dio_ti = norm(pos_gti - pos_o);
                nio_ti = (pos_gti - pos_o)/dio_ti;
                fio(i,:) = fio(i,:) + wij * Aso * norm(vel_g(i,:))/ti * exp(rio - dio/Bso) * nio_ti;
            else
                fio(i,:) = fio(i,:) + wij *Ao * exp((rio - dio) / Bo) * nio;
            end
        else
            fio(i,:) = fio(i,:) + wij * Ao * exp((rio - dio) / Bo) * nio;
            %         fio = 0;
        end
    end
 
end


F0 = fi0;
% Fe = fiw1 + fiw2 + fiw3 +wij * fir;
Fe = fiw + fio + fir + fig ;
for i = 1:grp.N
    ang(i) = atan2(e(i,2),e(i,1));
end
end