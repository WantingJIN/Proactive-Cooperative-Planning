function f = obj_MPC_init(Con,xr0,rob,TF,t_fine,t_int) % A start, B final ; X intermediate point
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

xr = xr0;
count =1;
for tspan = t_fine : t_fine : TF
    
    % Cost function f1: sum of the distance between the robot state and the goal
    % state in each time iteraction
    % Cost function f2: sum of the control input
    f2 = f2 + norm(Con(count,:))^2;
     
    
    %Update the robot state
    for tsint = 0 : t_int : t_fine - t_int
        dx_r = system_model_ROB(Con(count,:),xr);
        xr_new = xr' + dx_r * t_int;
        xr = xr_new';
    end
    
    count = count+1;
end
f1 = norm(rob.goal - xr(1:2))^2;

w1 = 20;
f1 = w1 * f1;


f = f1 + f2;
end