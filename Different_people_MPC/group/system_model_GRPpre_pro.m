function [ ds_g,fir ] = system_model_GRPpre_pro(sg,sr,Con,rob,grp)
% Calculate the differenciate term as a function of HSFM
% FORCES determination of the resulting force on each individual
% Input X=state of the system (robot or human)

%% Global Variable
kd = 500;
ko = 1;
%% Calculate the control input from social force

% Acting forces
[ F0,Fe, ang,fir,fig] = forces_grpSF_prepro(sg, sr,Con,rob,grp,grp.pregoal);
ds_g = zeros(grp.N,6);


for i = 1:grp.N
    
    FT(i,:) = F0(i,:) + Fe(i,:);
    % Magnitude of F0
    F_nV(i,:) = (sqrt(sum(abs(F0(i,:)).^2,2)));
    
    % desired theta
    thr(i) = mod(ang(i)', 2*pi);
    
    % actual theta
    th(i) = mod(sg(i,3), 2*pi);
    
    % angle to rotate
    ang(i)=th(i)-thr(i);
    td=[ang(i) ang(i)+2*pi ang(i)-2*pi];
    [~,I]=min(abs(td),[],2);
    
    % Calculate u_f
    u_f(i) = FT(i,:) * [cos(sg(i,3)) sin(sg(i,3))]';
    
    % Calculate u_o
    u_o(i) = ko * Fe(i,:) * [-sin(sg(i,3)) cos(sg(i,3))]' - kd * sg(i,5);
    
    % Calculate u_theta
    kl = 0.3;
    alfa = 3;
    kth = grp.J(i) * kl * F_nV(i,:);
    kom = grp.J(i) * (1 + alfa) * sqrt(kl * F_nV(i,:) / alfa);
    u_th(i) = (-kth * td(I) - kom * sg(i,6));
    
    %% Calculate the differenciate of the state
    
    ds_g(i,1) = sg(i,4) * cos(sg(i,3)) - sg(i,5) * sin(sg(i,3));
    ds_g(i,2) = sg(i,4) * sin(sg(i,3)) + sg(i,5) * cos(sg(i,3));
    ds_g(i,3) = sg(i,6);
    ds_g(i,4) = 1/grp.m(i) * u_f(i);
    ds_g(i,5) = 1/grp.m(i) * u_o(i);
    ds_g(i,6) = 1/grp.J(i) * u_th(i);
end

end