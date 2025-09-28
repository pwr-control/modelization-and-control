
%% SICME SN315X4 440kW data

pwr_nom = 440e3; % power of the motor
Vnom = 440; % phase to phase rated voltage
Inom = 793; % rated current
fnom = 34.0; % rated frequency Hz
cp = 2; % number of pole pairs
cosphi = 0.84;
no_load_rpm = fnom*60/cp;
slip = 12; % theoretical 17
load_rpm = 0;
if (slip ~= 0)
    load_rpm = no_load_rpm - slip;
end


%% nominal torque calculation
Tnom = 4584; % from datasheet
wr_rad = load_rpm/60*2*pi;
efficiency = 0.944;
pwr_load_test = 0;
if (pwr_nom == 0)
    pwr_nom = pwr_load_test*efficiency;
end
Tnom_calc = pwr_nom / wr_rad;
if (Tnom == 0)
    Tnom = Tnom_calc;
end
%% slip calculation
wnom = 2*pi*fnom; % synchronous speed rad /s
sl = (no_load_rpm - load_rpm) / no_load_rpm; % slip

%% stator resistance
DELTA_CONNECTION = 1;
PhaseResistance = 0.009;
if (DELTA_CONNECTION == 1)
    Rs = PhaseResistance/3;
else
    Rs = PhaseResistance;
end

%% first step Rr
Rr_min = Rs/10;
Rr_max = Rs*10;
deltaRr = Rs/100;
Tn = Tnom/2;
Rr = Rr_min;

%% No-load test --> Ls calc
I0 = 308;
Im = I0;
Ls = Vnom/sqrt(3)/I0/(2*pi*fnom);

%% Rotor-locked test --> Lleakage calc --> Lm = Ls - Lleakage
icc = Inom*4.5;
ucc = Vnom;
fcc = 50;
Zsigma = (ucc/sqrt(3))/icc;
Lsigma = sqrt(Zsigma^2-(Rs+Rr)^2)/(2*pi*fcc)
Lds = Lsigma/2;
Ldr = Lsigma-Lds;
Lm = Ls - Lds;
Lr = Lm + Ldr;

%% flux reference in SI
Im_bez = Im * sqrt(2);
fi_rot_ref = Im_bez * Lm;

%% steady state calculations
toll = 0.02;
while (((Tn > Tnom*(1+toll)) || (Tn < Tnom*(1-toll))) && (Rr < Rr_max))
    a = wnom*Lm*Rr/sl;
    b = wnom*Lm*a;
    c = (Rr/sl)^2+(wnom*Lr)^2;
    d = (Rr/sl)^2+wnom^2*Ldr*Lr;
    e = wnom^2*Lm*Lr;
 
    ZssRe = Rs + b/c;
    ZssIm = wnom*Lds + wnom*Lm*d/c;
    Zss_abs2 = ZssRe^2 + ZssIm^2;
    
    Is_ssRe = Vnom/sqrt(3)*ZssRe/Zss_abs2;
    Is_ssIm = -Vnom/sqrt(3)*ZssIm/Zss_abs2;
    
    Ir_ssRe = (a*Is_ssIm-e*Is_ssRe)/c;
    Ir_ssIm = -(a*Is_ssRe+e*Is_ssIm)/c;

    Ir_ss_abs2 = Ir_ssRe^2+Ir_ssIm^2;
    Pmech = 3*Ir_ss_abs2*Rr*(1-sl)/sl;
    Tn = Pmech / wr_rad;
    Rr = Rr+deltaRr;
end


Motor_name = string({'SICME SN315X4 440kW'});
comment1 = string({'Values with label "bez" mean always peak'});
comment2 = string({'Ubez means voltage peak phase'});
Ibez = Inom*sqrt(2);
Ubez = Vnom*sqrt(2/3);
Motor_param_name = string({'Inom = ', 'Vnom = ', 'Ibez = ', 'Ubez = ', ...
    'rpm = ','freq_nom = ','rpm_load = ', 'polepairs = ', 'Rs = ','Ls = ','Rr = ', 'Lr = ', ...
    'Lm = ', 'Im = ', 'Im_bez = ', 'Torque = ', 'cosphi = ', 'eff = ',...
    'ucc = ', 'icc = ', 'fcc = '})';
Motor_param = [Inom, Vnom, Ibez, Ubez, no_load_rpm, fnom, load_rpm, ...
    cp, Rs, Ls, Rr, Lr, Lm, Im, Im_bez, Tn, cosphi, efficiency,...
    ucc, icc, fcc];
Motor_param_unit = string({'A', 'V', 'A', 'V', 'rpm','Hz','rpm', '', 'Ohm',... 
    'H','Ohm','H','H','A','A','kNm', '', '','V','A','Hz'});
formatSpec = '%s %8.8f %s \r\n';
formatSpec_title = '%s \r\n';
formatSpec_comment = '%s \r\n';
formatSpec_separ = '%s \r\n \r\n';
id_im_data_motor = fopen('im_data_motor.txt','w+');
n=1;
fprintf(id_im_data_motor,formatSpec_title, Motor_name(n));
fprintf(id_im_data_motor,formatSpec_comment, comment1(n));
fprintf(id_im_data_motor,formatSpec_comment, comment2(n));
fprintf(id_im_data_motor,formatSpec_comment, '');
for n=1:length(Motor_param)
fprintf(id_im_data_motor,formatSpec, Motor_param_name(n), Motor_param(n), Motor_param_unit(n));
end

fclose(id_im_data_motor);
type im_data_motor.txt

