% movie speed, 1=30 fps
step=1;
rng(360);
cir=0:0.01:2*pi;

stepsize = 0.1;
tstep = 1 : t_fine : Tend;   % Time step for the optimization
tmov = 1 : stepsize : Tend; % Time step for playing optimization movie
rob.movS = interp1(tstep,rob.X,tmov); %Interpolation the value
hum.movS = interp1(tstep,hum.actX,tmov);

thor = 0 : t_fine : TF;  % Time step for planning in horizon time
thmov = 0 : stepsize : TF; % Time stap for playing predicted path movie


%%% MOVIE
scrsz = get(0,'ScreenSize');
vid=VideoWriter('simulation');
vid.FrameRate = 1/stepsize;% How many frames per second
open(vid);
h=figure('Color','w','Position',[1 1 800 800]);
N = 20;
color = [rand(N,1) rand(N,1) rand(N,1)];

for tt=1:step:size(rob.movS,1)
    % Plot of the walls
    for i=1:num_walls
        plot(map_walls(2*i-1:2*i,1),map_walls(2*i-1:2*i,2),'k','LineWidth',2);
        hold on
    end

    % Plot of the pedestrians represented as circles
    
    plot(rob.r*cos(cir) + rob.movS(tt,1),rob.r*sin(cir)+rob.movS(tt,2),'Color','r','LineWidth',2) % plot cerchi
    hold on
    plot(rob.r*cos(rob.movS(tt,3))+rob.movS(tt,1),rob.r*sin(rob.movS(tt,3))+rob.movS(tt,2),'ok')
    hold on
    
    rectangle('Position',[hum.actgoal 0.2 0.2],'FaceColor',[0 1 0])
    str={'Actual'};
    text(hum.actgoal(1)-0.5, hum.actgoal(2)-0.2,str,'fontsize',13);
    hold on
    rectangle('Position',[hum.pregoal 0.2 0.2],'FaceColor',[0 1 0])
    str={'Predicted'};
    text(hum.pregoal(1), hum.pregoal(2)-0.2,str,'fontsize',13);
    hold on
    plot(hum.r * cos(cir) + hum.movS(tt,1), hum.r * sin(cir) + hum.movS(tt,2),'Color','g','LineWidth',2) % plot cerchi
    hold on
    plot(hum.r * cos(hum.movS(tt,3)) + hum.movS(tt,1), hum.r * sin(hum.movS(tt,3)) + hum.movS(tt,2),'ok')
    hold on
    str = {'Human'};
    text(hum.movS(tt,1) + 0.5, hum.movS(tt,2),str,'fontsize',13);
    hold on
   
    
    if tt<size(rob.movS,1) - 1/t_fine-1
        for iter = 1: fix((tt - 1) * stepsize) + 1
            
            hum.premov{iter} = interp1(thor,hum.preX{iter},thmov); %Interpolation the value
            rob.planmov{iter} = interp1(thor,rob.planX{iter},thmov);
            
            plot(hum.premov{iter}(1,1),hum.premov{iter}(1,2),'o','LineWidth',1,'Color',color(iter,:));
            plot(rob.planmov{iter}(1,1),rob.planmov{iter}(1,2),'o','LineWidth',1,'Color',color(iter,:));
            hold on
            plot(hum.premov{iter}(:,1),hum.premov{iter}(:,2),'--','LineWidth',1,'Color',color(iter,:));
            plot(rob.planmov{iter}(:,1),rob.planmov{iter}(:,2),'--','LineWidth',1,'Color',color(iter,:));
            hold on
            str = {num2str(iter-1)};
            text(hum.premov{iter}(1,1),hum.premov{iter}(1,2),str,'Interpreter','Latex','fontsize',13);
            text(rob.planmov{iter}(1,1),rob.planmov{iter}(1,2),str,'Interpreter','Latex','fontsize',13);
        end
        if Method == 2
%             str = {'Reach the goal f1:' num2str(f1{iter})};
%             text(5.5,11,str,'Interpreter','Latex','fontsize',15);
%             str = {'Energy consuming f2:' num2str(f2{iter})};
%             text(5.5,9,str,'Interpreter','Latex','fontsize',15);
%             str = {'Interaction force on human f3:' num2str(f3{iter})};
%             text(5.5,7,str,'Interpreter','Latex','fontsize',15);
%             str = {'Directional cost f4:' num2str(f4{iter})};
%             text(5.5,5,str,'Interpreter','Latex','fontsize',15);
%             str = {'Time to collision cost f5:' num2str(f5{iter})};
%             text(5.5,3,str,'Interpreter','Latex','fontsize',15);
        end
        rectangle('Position',[rob.Goal{iter} 0.2 0.2],'FaceColor',[1 0 0])
        str={'Robot goal'};
        text(rob.Goal{iter}(1)-1, rob.Goal{iter}(2)-0.2,str,'fontsize',13);
        hold on
        
      
    else
        for iter = 1: fix((size(rob.movS,1) - 1/stepsize-1)*stepsize)+1
            
            hum.premov{iter} = interp1(thor,hum.preX{iter},thmov); %Interpolation the value
            rob.planmov{iter} = interp1(thor,rob.planX{iter},thmov);
            plot(hum.premov{iter}(1,1),hum.premov{iter}(1,2),'o','LineWidth',1,'Color',color(iter,:));
            plot(rob.planmov{iter}(1,1),rob.planmov{iter}(1,2),'o','LineWidth',1,'Color',color(iter,:));
            hold on
            plot(hum.premov{iter}(:,1),hum.premov{iter}(:,2),'--','LineWidth',1,'Color',color(iter,:));
            plot(rob.planmov{iter}(:,1),rob.planmov{iter}(:,2),'--','LineWidth',1,'Color',color(iter,:));
            hold on
                  str = {num2str(iter-1)};
            text(hum.premov{iter}(1,1),hum.premov{iter}(1,2),str,'Interpreter','Latex','fontsize',13);
            text(rob.planmov{iter}(1,1),rob.planmov{iter}(1,2),str,'Interpreter','Latex','fontsize',13);
        end
        if Method == 2
%             str = {'Reach the goal f1:' num2str(f1{iter})};
%             text(5.5,11,str,'Interpreter','Latex','fontsize',15);
%             str = {'Energy consuming f2:' num2str(f2{iter})};
%             text(5.5,9,str,'Interpreter','Latex','fontsize',15);
%             str = {'Interaction force on human f3:' num2str(f3{iter})};
%             text(5.5,7,str,'Interpreter','Latex','fontsize',15);
%             str = {'Directional cost f4:' num2str(f4{iter})};
%             text(5.5,5,str,'Interpreter','Latex','fontsize',15);
%             str = {'Time to collision cost f5:' num2str(f5{iter})};
%             text(5.5,3,str,'Interpreter','Latex','fontsize',15);
        end
        rectangle('Position',[rob.Goal{iter} 0.2 0.2],'FaceColor',[1 0 0])
        str={'Robot goal'};
        text(rob.Goal{iter}(1)-1, rob.Goal{iter}(2)-0.2,str,'fontsize',13);
        hold on
        
     
    end
    
    str = {'Robot'};
    text(rob.movS(tt,1) + 0.5, rob.movS(tt,2),str,'fontsize',13);
    hold on
    
    
    hold on
    
    for tp = 1:step:tt
        rob.path(tp,:) = rob.movS(tp,1:2);
        hum.path(tp,:) = hum.movS(tp,1:2);

    end
    
    plot(rob.path(:,1),rob.path(:,2),'LineWidth',1,'Color','r');
    hold on
    plot(hum.path(:,1),hum.path(:,2),'LineWidth',1,'Color','g');
    hold on

    hold on
    hold off
    axis equal
    axis off
    if Method==1
        cost = sprintf('%s%s%s%s%s','Objective function: ', num2str(w1), '*f1+' , num2str(w2) ,'*f2');
         title({['Reactive Planning'];cost;['Constraint: g1 g2'];['HSFM']},'Interpreter','Latex','FontSize',15)
    end
    if Method==2
        cost = sprintf('%s%s%s%s%s%s%s%s%s%s%s','Objective function: ', num2str(w1), '*f1+' , num2str(w2) ,'*f2+', num2str(w3), '*f3+', num2str(w4), '*f4+', num2str(w5), '*f5');
        title({['Cooperative Planning'];cost;['Constraint: g1 g2 g3'];['EHSFM with explicit collision prediction']},'Interpreter','Latex','FontSize',15)
    end
    M=getframe(h);
    writeVideo(vid,M);
end
close(vid)



