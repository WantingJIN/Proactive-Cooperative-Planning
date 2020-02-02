%Proactive planning for robot in one time iteraction
%Predict human trajectory by using HSFM with explicit collision avoidance
%Control the robot motion by using MPC
%-------------------------------------------------------------------------------------------------%
clear all
clc
close all
%% Initialization
% Simulation time
Tend = 11;
TF =6; % Horizon time
t_upd = 1; % time step for update the robot and human state from sensor
t_fine = 1/10; % timing accuracy in the movie (framerate=30)
t_int = 1/20;
% Environment Parameter
map_walls = map_def;
[dnum_walls, ~] = size(map_walls);
% Number of walls
num_walls = dnum_walls/2;

% Individual characteristics

% Group characteristics
% Number of menbers in a group
grp.N = 2;
% represent the relationship between the group
grp.kai = 20;
% Initial distance between two person in a group
grp.l0 = 2;
% Thereshold distance to take two people as a group
grp.ld = 3;

% Radius
rob.r = 0.3;
hum.r = 0.3;
grp.r = [0.3;0.2];

% Mass
rob.m = 60;
hum.m = 75;
grp.m = [65;75];

% Inertia matrix
rob.J =0.5*rob.r^2;
hum.J = 0.5*hum.r^2;
grp.J = 0.5 * grp.r .^ 2;

% Initial state
% Initial velocity in body frame
rob.v0 = 0 * ones(1,2);
hum.v0 = 0 * ones(1,2);
grp.v0 = 0 * ones(2,2);

% Initial heading angle
rob.th =- pi/2;
hum.th = -pi/2;
grp.th = [pi/2;pi/2];


% Initial angular velocity
hum.omg = 0;
grp.omg = [0;0];

% Desired speed
hum.vd =1.5;
grp.vd = [1.5;1.5];

% Starting point
rob.sp = [2.5 15];
hum.sp = [2.5 15];
grp.sp = [2.5-grp.l0/2 1; 2.5+grp.l0/2 1];
% Goal position
rob.goal = [2.5 1];
hum.pregoal = [2.5 1];
hum.actgoal = [1.5 1];
grp.pregoal = [2.5-grp.l0/2 15; 2.5+grp.l0/2 15];
grp.actgoal = [2.5-grp.l0/2 15; 2.5+grp.l0/2 15];
% State
rob.x =[rob.sp rob.th];
hum.x =[hum.sp hum.th hum.v0 hum.omg];
grp.x = [grp.sp grp.th grp.v0 grp.omg];

rob.X = rob.x;
hum.actX = hum.x;

for i=1:grp.N
    grp.actX{i}=grp.x(i,:);
end

%% Motion Predictive Control
for iter = 1 : t_upd : Tend-1

    % Determin which kind of planning strategy to use
    % Method=1 reactive; Method = 2 proactive
    
    Method = 2;
    
    con0 = zeros(1/t_fine*TF,2);   %initial solution
    con0(:,1) =0;
    
    LB = -ones(1/t_fine*TF,2);
    LB(:,1) = -0.1;
    UB = ones(1/t_fine*TF,2);
    UB(:,1) = 2;
    
    
    xr = rob.x;
    xg = grp.x;
    %% Reactive Planning
    
    if Method==1
        
        options = optimset('Display','iter',...
            'TolX',1.e-8,...
            'TolFun',1.e-8,...
            'MaxIter',5000,...
            'MaxfunEvals',50000); % change different algorithm can get different results
        
        
        % Predict the hum trajectory
        rob.Goal{iter}=rob.goal;
        for i = 1:grp.N
            grp.preX{iter}{i} = xg(i,:);
        end
        
        for tspan = t_fine: t_fine : TF % The control input stay constant within t_fine
            for tsint = 0: t_int: t_fine - t_int % The intergal step with in one t_fine
                dx_g = system_model_GRPpre_reac(xg, grp);
                xg_new = xg + dx_g * t_int;
                xg = xg_new;
            end
            for i = 1:grp.N
                grp.preX{iter}{i} = [grp.preX{iter}{i};xg(i,:)];
            end
        end
        %         grp.actX = grp.preX;
        
        disp('Reactive Planning')
        Con_reac = fmincon(@(con0)obj_MPC_reac(con0, rob.x, rob, TF, t_fine, t_int),con0,[],[],[],[],LB,UB, @(con0)const_MPC_reac(con0, rob.x, grp.preX{iter}, rob, grp,TF,t_fine,t_int),options);
        
        [f,f1,f2,w1,w2]=obj_MPC_reac(con0, rob.x, rob, TF, t_fine, t_int);
        
        % Calculate the planned path for robot in next horizon time
        count =1;
        rob.planX{iter} = xr;
        
        for tspan=t_fine : t_fine : TF
            for tsint = 0 : t_int : t_fine - t_int
                % Calculate the differentiate of the robot system
                dx_r = system_model_ROB(Con_reac(count,:),xr);
                xr_new = xr' + dx_r * t_int;
                %Update the state forand robot
                
                xr = xr_new';
            end
            rob.planX{iter} = [rob.planX{iter};xr];
            count = count+1;
        end
        Con = Con_reac;
    end
    %% Proactive Planning
    % Optimization of rob control input
    if Method == 2
        
        options = optimset('Display','iter',...
            'TolX',1.e-8,...
            'TolFun',1.e-8,...
            'MaxIter',5000,...
            'MaxfunEvals',50000,...
            'UseParallel','always'); % change different algorithm can get different results
        disp('Proactive-cooperative planning');
        tic
        Con_pro = fmincon(@(con0)obj_MPC_pro(con0,rob.x,grp.x,rob,grp,TF,t_fine,t_int),con0,[],[],[],[],LB,UB, @(con0)const_MPC_pro(con0,rob.x,grp.x,rob,grp,TF,t_fine,t_int),options);
        toc
        %Con=con0;
        [f{iter}, f1{iter}, f2{iter}, f3{iter}, f4{iter}, f5{iter},f6{iter},w1,w2,w3,w4,w5,w6]=obj_MPC_pro(Con_pro,rob.x,grp.x,rob,grp,TF,t_fine,t_int);
        
        
        %% Update the rob and human predicted state
        rob.Goal{iter} = rob.goal;
        xr = rob.x;
        rob.planX{iter} = xr;
        xg = grp.x;
        
        for i = 1:grp.N
            grp.preX{iter}{i} = xg(i,:);
        end
        
        count = 1;
        for tspan=t_fine : t_fine : TF
            for tsint = 0 : t_int : t_fine - t_int
                % Calculate the differentiate of the robot system
                dx_r = system_model_ROB(Con_pro(count,:),xr);
                xr_new = xr' + dx_r * t_int;
                
                % Calculate the control input of human system by using updated robot
                % state
                [dx_g,~] = system_model_GRPpre_pro(xg,xr,Con_pro(count,:),rob,grp);
                xg_new = xg + dx_g * t_int;
                
                %Update the state for human and robot
                xg = xg_new;
                xr = xr_new';
            end
            rob.planX{iter} = [rob.planX{iter};xr];
            for i=1:grp.N
                grp.preX{iter}{i} = [grp.preX{iter}{i};xg(i,:)];
            end
            count = count+1;
        end
        Con = Con_pro;
    end
    % Update the rob state, only execute the control input of  first
    % t_upd, and simulate the real human reaction towards robot
    % motion(different from predicted one)
    xr = rob.x;
    xg = grp.x;
    count = 1;
    
    
    for tspan= t_fine : t_fine : t_upd
        for tsint = 0: t_int : t_fine-t_int
            dx_r = system_model_ROB(Con(count,:),xr);
            xr_new = xr' + dx_r * t_int;
            
            % Update the hum state from sensor information at each time iterval
            [dx_g,fir] = system_model_GRPact_pro(xg,xr,Con(count,:),rob,grp);
            xg_new = xg + dx_g * t_int;
            
            xg = xg_new;
            xr = xr_new';
        end
        
        rob.X = [rob.X;xr];
        for i = 1:grp.N
            grp.actX{i} = [grp.actX{i};xg(i,:)];
        end
        count = count+1;
    end
    
    %State feedback for MPC
    rob.x = xr_new';
    grp.x = xg_new;
    
end


moviepaly