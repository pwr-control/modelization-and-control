clear all
close all
clc
beep off

fPWM = 3100;
SYS_TIMEBASE_FACTOR = 8;
ts_fast = 1/fPWM/2;
ts_slow = ts_fast*SYS_TIMEBASE_FACTOR;
tc = ts_fast/2;
z=tf('z',ts_slow);
s=tf('s');

use_integral_state_feedback_control = 1;
use_pi_control = 1-use_integral_state_feedback_control;
torque_disturbance = 0.1;

model2_parameters;
% model1_parameters;

omega_ref_pu = 1;

time_start = 0.1; 
stop = 50;
slope_rise = 1;
slope_down = 1;
simlength = 1*(stop+100);
% simlength = 1*(stop);
Ns_slow = floor(simlength/ts_slow);
Ns_fast = floor(simlength/ts_fast);
Nc = floor(simlength/tc);

wcut2 = 2*pi*30;
power_conv = 10^(0/20)*1/(s/wcut2+1);
power_conv_d = c2d(power_conv,ts_slow)*z^-1;
% figure;
% set(cstprefs.tbxprefs,'FrequencyUnits','Hz');
% bode(power_conv_d);
% title('converter eq transfer function');
% grid;

wcut3 = 2*pi*20;
lowpassfilter = 10^(0/20)*1/(s/wcut3+1);
lowpassfilter_d = c2d(lowpassfilter,ts_slow)*z^-1;
% figure;
% set(cstprefs.tbxprefs,'FrequencyUnits','Hz');
% bode(lowpassfilter_d);
% title('low pass fst order ref filter');
% grid;

wcut4 = 2*pi*20;
lowpassfilter_snd = 10^(0/20)*1/(s/wcut4+1)^2;
lowpassfilter_snd_d = c2d(lowpassfilter_snd,ts_slow)*z^-1;
% figure;
% set(cstprefs.tbxprefs,'FrequencyUnits','Hz');
% bode(lowpassfilter_snd_d);
% title('low pass snd order ref filter');
% grid;

Jm_model = Jm_max;
Jl_model = Jl_max;
% Jm_model = Jm_min;
% Jl_model = Jl_min;
Ktheta_model = Ktheta

J1 = 1/2*Jm_model*omega_nom/torque_nom
J2 = 1/2*Jl_model*omega_nom/torque_nom
b1 = Dm*omega_nom/torque_nom
b2 = Dl*omega_nom/torque_nom
b_theta = Dtheta*omega_nom/torque_nom
k_theta = Ktheta_model/torque_nom

A = [-(b1+b_theta)/J1 b_theta/J1 -1/J1; ...
    b_theta/J2 -(b2+b_theta)/J2 1/J2;...
    k_theta -k_theta 0];
B = [1/J1 0 0]';

E = [0 -1/J2 0]';
C = [1 0 0];


%% PI + state feedback
k1 = 1;
f1 = 1*f0_min*k1;
f2 = 1*f0_min*k1;
f3 = 1*f0_min*k1;
p1 = -2*pi*f1;
p2 = -2*pi*f2;
p3 = -2*pi*f3;

% disable_sfc = 0;
% if (disable_sfc == 0)
% Ksf_t = round(acker(A,B,[p1, p2, p3]), 2)
% Ksf = [Ksf_t, -1]
% kp = 2;
% ki = 45;
% else
% kp = 2;
% ki = 12.5;  
% Ksf = [0, 0, 0, 0];
% end

Q = [1 0 0; 0 10 0; 0 0 1]*1;
R = 0.1;
disable_sfc = 0;
if (disable_sfc == 0)
Ksf_t = lqr(A,B,Q,R)
Ksf = [Ksf_t, -1];
kp = 2;
ki = 45;
else
kp = 2;
ki = 12.5;  
Ksf = [0, 0, 0, 0];
end

%% State Observer
% k2 = 2;
% f1 = 1*f0_min*k2;
% f2 = 1*f0_min*k2;
% f3 = 1*f0_min*k2;
% p1 = -f1*2*pi;
% p2 = -f2*2*pi;
% p3 = -f3*2*pi;
% L = round(acker(A',C',[p1,p2,p3])',2)
% Ad = eye(3) + A*ts_slow; 
% Bd = B*ts_slow;
% Ed = E*ts_slow;

%% State Observer with load estimator
Ale = [-(b1+b_theta)/J1 b_theta/J1 -1/J1 0; ...
       b_theta/J2 -(b2+b_theta)/J2 1/J2 1/J2;...
       k_theta -k_theta 0 0;
       0 0 0 0];
Ble = [1/J1 0 0 0]';
Cle = [1 0 0 0];

Aled = eye(4) + Ale*ts_slow;
Bled = Ble*ts_slow;

% k2 = 2;
% f1 = 1*f0_min*k2;
% f2 = 1*f0_min*k2;
% f3 = 1*f0_min*k2;
% f4 = 1*f0_min*k2;
% p1 = -f1*2*pi;
% p2 = -f2*2*pi;
% p3 = -f3*2*pi;
% p4 = -f4*2*pi;
% Lle = acker(Ale',Cle',[p1,p2,p3,p4])'

Qle = [5 0 0 0; 0 5 0 0; 0 0 5 0; 0 0 0 2]*1e2;
Rle = 1;
Lle = (lqr(Ale',Cle',Qle,Rle))'


torque_m_pu_lim = 1.4;
load_ratio = 1;

% Jm_plant = Jm_min;
% Jl_plant = Jl_min;

Jm_plant = Jm_max;
Jl_plant = Jl_max;

Ktheta_plant = Ktheta
Dm_plant = 0;
Dl_plant = 0;
Dtheta_plant = 0;

model = 'flexible_shaft_ctrl';
open_system(model);
Simulink.importExternalCTypes(model,'Names',{'adv_speed_ctrl_pisf_t'});
Simulink.importExternalCTypes(model,'Names',{'adv_speed_ctrl_pisf_output_t'});
Simulink.importExternalCTypes(model,'Names',{'adv_speed_ctrl_pisfle_t'});
Simulink.importExternalCTypes(model,'Names',{'adv_speed_ctrl_pisfle_output_t'});

% Lle = [3.04; 2.22; -4.63; 3.01]
% % Ksf = [11 50 52.5 -1]
% Ksf = [2.62 -0.84 5 -1.0]





