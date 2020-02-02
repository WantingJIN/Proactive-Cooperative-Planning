% movie speed, 1=30 fps
step=1;
rng(360);
cir=0:0.01:2*pi;

stepsize = 0.1;
tstep = 1 : t_fine : Tend;   % Time step for the optimization
tmov = 1 : stepsize : Tend; % Time step for playing optimization movie
thor = 0 : t_fine : TF;  % Time step for planning in horizon time
thmov = 0 : stepsize : TF; % Time stap for playing predicted path movie
rob.movS = interp1(tstep,rob.X,tmov); %Interpolation the value
% hum.movS = interp1(tstep,hum.actX,tmov);
for i = 1:grp.N
    %     grp.movPreS{i} = interp1(thor,grp.PreS{i},thmov);
    grp.movActX{i} = interp1(tstep,grp.actX{i},tmov);
end

%%% MOVIE
scrsz = get(0,'ScreenSize');
vid=VideoWriter('simulation');
vid.FrameRate = 1/stepsize;% How many frames per second
open(vid);
h=figure('Color','w','Position',[1 1 900 900]);
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
    
    %     rectangle('Position',[rob.Goal{iter} 0.2 0.2],'FaceColor',[1 0 0])
    %     str={'Robot goal'};
    %     text(rob.goal(1)-1, rob.goal(2)-0.2,str,'fontsize',13);
    for i = 1:grp.N
        rectangle('Position',[grp.actgoal(i,:) 0.2 0.2],'FaceColor',[0 1 0])
        str={'Actual'};
        text(grp.actgoal(i,1)-0.5, grp.actgoal(i,2)-0.2,str,'fontsize',13);
        hold on
        rectangle('Position',[grp.pregoal(i,:) 0.2 0.2],'FaceColor',[0 1 0])
        str={'Predicted'};
        text(grp.pregoal(i,1), grp.pregoal(i,2)-0.2,str,'fontsize',13);
        hold on
    end
    for i = 1:grp.N
        plot(grp.r(i) * cos(cir) + grp.movActX{i}(tt,1), grp.r(i) * sin(cir) + grp.movActX{i}(tt,2),'Color','m','LineWidth',2) % plot cerchi
        hold on
        plot(grp.r(i) * cos(grp.movActX{i}(tt,3)) + grp.movActX{i}(tt,1), grp.r(i) * sin(grp.movActX{i}(tt,3)) + grp.movActX{i}(tt,2),'ok')
        hold on
    end
    str = {'Group'};
    text(grp.movActX{1}(tt,1) + 0.5, grp.movActX{1}(tt,2),str,'Interpreter','Latex','fontsize',13);
    hold on
    
    
    if tt<size(rob.movS,1) - 1/t_fine-1
        for iter = 1: fix((tt - 1) * stepsize) + 1
            for i = 1:grp.N
                grp.premov{iter}{i} = interp1(thor,grp.preX{iter}{i},thmov); %Interpolation the value
                plot(grp.premov{iter}{i}(1,1),grp.premov{iter}{i}(1,2),'o','LineWidth',1,'Color',color(iter,:));
                hold on
                plot(grp.premov{iter}{i}(:,1),grp.premov{iter}{i}(:,2),'--','LineWidth',1,'Color',color(iter,:));
            end
            rob.planmov{iter} = interp1(thor,rob.planX{iter},thmov);
            plot(rob.planmov{iter}(1,1),rob.planmov{iter}(1,2),'o','LineWidth',1,'Color',color(iter,:));
            hold on
            plot(rob.planmov{iter}(:,1),rob.planmov{iter}(:,2),'--','LineWidth',1,'Color',color(iter,:));
            hold on
        end
        
        if Method == 2
            str = {'Reach the goal f1:' num2str(f1{iter})};
            text(5.5,11,str,'Interpreter','Latex','fontsize',15);
            str = {'Energy consuming f2:' num2str(f2{iter})};
            text(5.5,9,str,'Interpreter','Latex','fontsize',15);
            str = {'Interaction force on human f3:' num2str(f3{iter})};
            text(5.5,7,str,'Interpreter','Latex','fontsize',15);
            str = {'Directional cost f4:' num2str(f4{iter})};
            text(5.5,5,str,'Interpreter','Latex','fontsize',15);
            str = {'Time to collision cost f5:' num2str(f5{iter})};
            text(5.5,3,str,'Interpreter','Latex','fontsize',15);
            str = {'Group potential energy f6:' num2str(f6{iter})};
            text(5.5,1,str,'Interpreter','Latex','fontsize',15);
        end
        rectangle('Position',[rob.Goal{iter} 0.2 0.2],'FaceColor',[1 0 0])
        str={'Robot goal'};
        text(rob.Goal{iter}(1)-1, rob.Goal{iter}(2)-0.2,str,'fontsize',13);
        hold on
    else
        for iter = 1: fix((size(rob.movS,1) - 1/stepsize-1)*stepsize)+1
            
            for i = 1:grp.N
                grp.premov{iter}{i} = interp1(thor,grp.preX{iter}{i},thmov); %Interpolation the value
                plot(grp.premov{iter}{i}(1,1),grp.premov{iter}{i}(1,2),'o','LineWidth',1,'Color',color(iter,:));
                hold on
                plot(grp.premov{iter}{i}(:,1),grp.premov{iter}{i}(:,2),'--','LineWidth',1,'Color',color(iter,:));
            end
            rob.planmov{iter} = interp1(thor,rob.planX{iter},thmov);
            
            plot(rob.planmov{iter}(1,1),rob.planmov{iter}(1,2),'o','LineWidth',1,'Color',color(iter,:));
            hold on
            
            plot(rob.planmov{iter}(:,1),rob.planmov{iter}(:,2),'--','LineWidth',1,'Color',color(iter,:));
            hold on
            
        end
        
        if Method == 2
            str = {'Reach the goal f1:' num2str(f1{iter})};
            text(5.5,11,str,'Interpreter','Latex','fontsize',15);
            str = {'Energy consuming f2:' num2str(f2{iter})};
            text(5.5,9,str,'Interpreter','Latex','fontsize',15);
            str = {'Interaction force on human f3:' num2str(f3{iter})};
            text(5.5,7,str,'Interpreter','Latex','fontsize',15);
            str = {'Directional cost f4:' num2str(f4{iter})};
            text(5.5,5,str,'Interpreter','Latex','fontsize',15);
            str = {'Time to collision cost f5:' num2str(f5{iter})};
            text(5.5,3,str,'Interpreter','Latex','fontsize',15);
            str = {'Group potential energy f6:' num2str(f6{iter})};
            text(5.5,1,str,'Interpreter','Latex','fontsize',15);
        end
        rectangle('Position',[rob.Goal{iter} 0.2 0.2],'FaceColor',[1 0 0])
        str={'Robot goal'};
        text(rob.Goal{iter}(1)-1, rob.Goal{iter}(2)-0.2,str,'fontsize',13);
        hold on
        
        %         plot(hum.premov{iter}(:,1),hum.premov{iter}(:,2),'--','LineWidth',1,'Color',color(iter,:));
        %         plot(rob.planmov{iter}(:,1),rob.planmov{iter}(:,2),'--','LineWidth',1,'Color',color(iter,:));
        %         hold on
    end
    
    str = {'Robot'};
    text(rob.movS(tt,1) + 0.5, rob.movS(tt,2),str,'fontsize',13);
    hold on
    
    
    hold on
    
    for tp = 1:step:tt
        rob.path(tp,:) = rob.movS(tp,1:2);
        for i = 1:grp.N
            grp.path{i}(tp,:) = grp.movActX{i}(tp,1:2);
        end
    end
    
    plot(rob.path(:,1),rob.path(:,2),'LineWidth',1,'Color','r');
    hold on
    for i = 1:grp.N
        plot(grp.path{i}(:,1),grp.path{i}(:,2),'LineWidth',1,'Color','m');
        hold on
    end
    
    hold on
    hold off
    axis equal
    axis off
    if Method==1
        title({['Reactive MPC f1+f2 ; g1 g3 g6'];['HSFM Simulation']},'Interpreter','Latex','FontSize',15)
    end
    if Method==2
        cost = sprintf('%s%s%s%s%s%s%s%s%s%s%s%s%s','Objective function: ', num2str(w1), '*f1+' , num2str(w2) ,'*f2+', num2str(w3), '*f3+', num2str(w4), '*f4+', num2str(w5), '*f5+',num2str(w6), '*f6');
        title({['Cooperative Planning'];cost;['Constraint: g1 g2 g3'];['HSFM with explicit collision prediction']},'Interpreter','Latex','FontSize',15)
    end
    M=getframe(h);
    writeVideo(vid,M);
end
close(vid)



