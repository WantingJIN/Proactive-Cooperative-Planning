function [f,f1,f2,w1,w2] = obj_MPC_reac(Con, x_r, rob, TF, t_fine,t_int) % A start, B final ; X intermediate point
% Object funtion for the multi-objective function
% Proactive planning should both contained the cost function for robot trajectory and huaman trajetory
% x_r x_p is the state of robot and pedestrian update from the sensor
% information in each time iteration
%% Simulation time

f1 = 0;
f2 = 0;

count =1;
for tspan=0:t_fine:TF-t_fine
    
    
    % Cost function f1: sum of the distance between the robot state and the goal
    % state in each time iteraction
    f1 = f1 + norm(rob.goal - x_r(1:2))^2;
    
    % Cost function f2: sum of the control input
    f2 = f2 + norm(Con(count,:))^2;
    
    for tsint = 0 : t_int : t_fine - t_int
        %Update the robot state
        dx_r = system_model_ROB(Con(count,:),x_r);
        xr_new = x_r' + dx_r * t_int;
        x_r = xr_new';
    end
    count = count+1;
end

w1 = 1;
w2 = 1;
f1 = w1*f1;
f2 = w2*f2;
f = f1 + f2;
end