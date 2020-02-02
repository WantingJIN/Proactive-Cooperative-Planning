function [ dX ] = system_model_ROB(Con,X)
% Calculate the differenciate term as a function of HSFM
% FORCES determination of the resulting force on each individual
% Input X=state of the system (robot or human)



%% Calculate the differenciate of the state
dX = zeros(3,1);
dX(1)=Con(1)*cos(X(3));
dX(2)=Con(1)*sin(X(3));
dX(3)=Con(2);

end