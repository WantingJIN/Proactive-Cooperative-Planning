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
TF = 6; % Horizon time
t_upd = 1; % time step for update the robot and human state from sensor
t_fine = 1/10; % timing accuracy in the movie (framerate=30)
t_int = 1/20;

% Environment Parameter
map_walls = map_def;
[dnum_walls, ~] = size(map_walls);
% Number of walls
num_walls = dnum_walls/2;

% Individual characteristics
% Radius
rob.r = 0.3;
hum.r = 0.3;
% Mass
rob.m = 60;
hum.m = 75;
% Inertia matrix
rob.J = 0.5 * rob.r^2;
hum.J = 0.5 * hum.r^2;
% Initial state
% Initial velocity in body frame
rob.v0 = 0 * ones(1,2);
hum.v0 = 0 * ones(1,2);
% Initial heading angle
rob.th = pi/2;
hum.th = -pi/2;
% Initial angular velocity
hum.omg = 0;
% Desired speed
hum.vd =1.5;
% Starting point
rob.sp = [2.5 1];
hum.sp = [2.5 15];
% Goal position
rob.goal = [2.5 15];
hum.pregoal = [2.5 1];
hum.actgoal = [2.5 1];

% State
rob.x =[rob.sp rob.th];
hum.x =[hum.sp hum.th hum.v0 hum.omg];

rob.X = rob.x;
hum.actX = hum.x;


%% Motion Predictive Control
for iter = 1 : t_upd : Tend-1
    % Determin which kind of planning strategy to use
%     Method=1 reactive; Method = 2 proactive

    Method{iter}=2;
 
    %% Reactive Planning
    
    if Method{iter}==1
    con0 = zeros(1/t_fine*TF,2);   %initial solution
    con0(:,1) = 1;
    
    LB = -ones(1/t_fine*TF,2);
    LB(:,1) = -0.1;
    UB = ones(1/t_fine*TF,2);
     UB(:,1) = 2;
    
        options = optimset('Display','iter',...
            'TolX',1.e-8,...
            'TolFun',1.e-8,...
            'MaxIter',5000,...
            'MaxfunEvals',50000,...
            'UseParallel','always'); % change different algorithm can get different results
        
        
        % Predict the hum trajectory
        rob.Goal{iter}=rob.goal;
        xh = hum.x;
        hum.preX{iter} = xh;
        count = 1;
        for tspan=0:t_fine:TF-t_fine
            for tsint = 0: t_int: t_fine-t_int
                dx_h = system_model_HUMpre_reac(xh,hum);
                xh_new = xh' + dx_h * t_int;
                xh = xh_new';
            end
            hum.preX{iter} = [hum.preX{iter};xh_new'];
            count = count+1;
        end
        
        disp('Reactive Planning')
        Con_reac = fmincon(@(con0)obj_MPC_reac(con0,rob.x,rob,TF,t_fine,t_int),con0,[],[],[],[],LB,UB, @(con0)const_MPC_reac(con0,rob.x,hum.preX{iter},rob,hum,TF,t_fine,t_int),options);
         [f{iter}, f1{iter}, f2{iter},w1,w2]=obj_MPC_reac(Con_reac,rob.x,rob,TF,t_fine,t_int);
        % Calculate the robot plan in the future horizon time
        count=1;
        xr = rob.x;
        rob.planX{iter} = xr;
        for tspan=t_fine : t_fine : TF
            for tsint = 0 : t_int : t_fine - t_int
                % Calculate the differentiate of the robot system
                dx_r = system_model_ROB(Con_reac(count,:),xr);
                xr_new = xr' + dx_r * t_int;
                
                %Update the state for robot
                xr = xr_new';
            end
            rob.planX{iter} = [rob.planX{iter};xr];
            count = count+1;
        end
        Con = Con_reac;
    end
    
    
    
    %% Proactive Planning
    % Optimization of rob control input
    if Method{iter} == 2
    con0 = zeros(1/t_fine*TF,2);   %initial solution
    con0(:,1) = 0;
    
    LB = -ones(1/t_fine*TF,2);
    LB(:,1) = -0.1;
    UB = ones(1/t_fine*TF,2);
    UB(:,1) = 2;
        options = optimset('Display','iter',...
            'TolX',1.e-8,...
            'TolFun',1.e-8,...
            'MaxIter',5000,...
            'MaxfunEvals',50000,...
            'UseParallel','always'); % change different algorithm can get different results
        tic
        disp('Proactive-Cooperative planning')
        Con_pro = fmincon(@(con0)obj_MPC_pro(con0,rob.x,hum.x,rob,hum,TF,t_fine,t_int),con0,[],[],[],[],LB,UB, @(con0)const_MPC_pro(con0,rob.x,hum.x,rob,hum,TF,t_fine,t_int),options);
        toc
        %Con=con0;
        [f{iter}, f1{iter}, f2{iter}, f3{iter}, f4{iter}, f5{iter},w1,w2,w3,w4,w5]=obj_MPC_pro(Con_pro,rob.x,hum.x,rob,hum,TF,t_fine,t_int);
        
        
        %% Update the rob and human state
        rob.Goal{iter} = rob.goal;
        
        xr = rob.x;
        rob.planX{iter} = xr;
        xh = hum.x;
        hum.preX{iter} = xh;
        count = 1;
        % Calcularte the robot plan and predicted human reaction in the h time
        for tspan=t_fine : t_fine : TF
            for tsint = 0 : t_int : t_fine - t_int
                % Calculate the differentiate of the robot system
                dx_r = system_model_ROB(Con_pro(count,:),xr);
                xr_new = xr' + dx_r * t_int;
                
                % Calculate the control input of human system by using updated robot
                % state
                [dx_h,~] = system_model_HUMpre_pro(xh,xr,Con_pro(count,:),rob,hum);
                xh_new = xh' + dx_h * t_int;
                
                %Update the state for human and robot
                xh = xh_new';
                xr = xr_new';
            end
            rob.planX{iter} = [rob.planX{iter};xr];
            hum.preX{iter} = [hum.preX{iter};xh];
            count = count+1;
        end
        Con = Con_pro;
    end
    
    
    % Update the rob state, only execute the control input of  first
    % t_upd, and simulate the real human reaction towards robot
    % motion(different from predicted one)
    xr = rob.x;
    xh = hum.x;
    count = 1;
  
    for tspan= t_fine : t_fine : t_upd
        % Based on the newest actual humam state to predict the human next
        % reaction wrt different assumptions
        % integral process
        for tsint = 0: t_int : t_fine-t_int
            dx_r = system_model_ROB(Con(count,:),xr);
            xr_new = xr' + dx_r * t_int;
           
            % Update the hum actual state from sensor information at each time iterval
            [dx_h,~] = system_model_HUMact_pro(xh,xr,Con(count,:),rob,hum);
%             [dx_h] = system_model_HUMact_reac(xh,hum);
            xh_new = xh' + dx_h * t_int;
            
            xh = xh_new';
            xr = xr_new';
        end
        
        rob.X = [rob.X;xr];
        hum.actX = [hum.actX;xh];
        count = count+1;
    end

    %State feedback for MPC
    rob.x = xr_new';
    hum.x = xh_new';
    %     if norm(rob.x(1:2) - rob.goal) < 7
    %         rob.goal(2) = rob.goal(2) + 7;
    %     end
    
    
    
end
moviepaly