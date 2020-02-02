function  [ds_h,fir] = system_model_HUMact_pro(sh,sr,Con,rob,hum)
% Calculate the differenciate term as a function of HSFM
% FORCES determination of the resulting force on each individual
% Input X=state of the system (robot or human)

%% HSFM Parameters
kd = 500;
ko = 1;

%% Calculate the control input from social force

% Acting forces
[F0,Fe,ang,fir]=forces_SF_actpro(sh,sr,Con,rob,hum,hum.actgoal);
FT = F0 + Fe;

% Magnitude of F0
F_nV = (sqrt(sum(abs(F0).^2,2)));

% desired theta
thr = mod(ang',2*pi);

% actual theta
th = mod(sh(3),2*pi);

% angle to rotate
ang = th - thr;
td = [ang ang+2*pi ang-2*pi];
[~,I] = min(abs(td),[],2);

% Calculate u_f
u_f = FT*[cos(sh(3)) sin(sh(3))]';
% Calculate u_o
u_o = ko*Fe*[-sin(sh(3)) cos(sh(3))]'-kd*sh(5);
% Calculate u_theta 
kl = 0.3;
alfa = 3;
kth = hum.J * kl * F_nV; 
kom = hum.J * (1 + alfa) * sqrt(kl * F_nV / alfa);
% u_th = (-kth * ang - kom * sh(6));
u_th =(-kth * td(I) - kom * sh(6));
%% Calculate the differenciate of the state
ds_h = zeros(6,1);
ds_h(1) = sh(4) * cos(sh(3)) - sh(5) * sin(sh(3));
ds_h(2) = sh(4) * sin(sh(3)) + sh(5) * cos(sh(3));
ds_h(3) = sh(6);
ds_h(4) = 1/hum.m * u_f;
ds_h(5) = 1/hum.m * u_o;
ds_h(6) = 1/hum.J * u_th;
end