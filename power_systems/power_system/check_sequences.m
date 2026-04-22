% clear all
close all
clc

tc = 1e-4;
T = 20e-3;
time = 0:tc:3*T-tc;
theta = 2*pi/T*time;

Uu = cos(theta);
Uv = 0.25*cos(theta-2*pi/3);
Uw = cos(theta-4*pi/3);

Ualpha = 2/3*(Uu - 1/2*Uv - 1/2*Uw);
Ubeta  = 1/sqrt(3)*(Uv - Uw);

Uxi_p  =  Ualpha.*cos(theta) + Ubeta.*sin(theta);
Ueta_p = -Ualpha.*sin(theta) + Ubeta.*cos(theta);

Uxi_n  =  Ualpha.*cos(theta) - Ubeta.*sin(theta);
Ueta_n = Ualpha.*sin(theta) + Ubeta.*cos(theta);

figure; plot(time, Uxi_p, time, Ueta_p, time, Uxi_n, time, Ueta_n); grid on
legend('uxi_p','ueta_p','uxi_n', 'ueta_n')


deepPOSxi = 0.25;
deepPOSeta = 0;
deepNEGxi = 0.75;
deepNEGeta = 0;

Ualpha_p = deepPOSxi * cos(theta) - deepPOSeta * sin(theta);
Ubeta_p = deepPOSxi * sin(theta) + deepPOSeta * cos(theta);

Ualpha_n = deepNEGxi * cos(theta) + deepNEGeta * sin(theta);
Ubeta_n = -deepNEGxi * sin(theta) + deepNEGeta * cos(theta);

Ualpha = Ualpha_p + Ualpha_n;
Ubeta = Ubeta_p + Ubeta_n;

Uu = Ualpha;
Uv = -0.5*Ualpha + sqrt(3)/2*Ubeta;
Uw = -0.5*Ualpha - sqrt(3)/2*Ubeta;

figure; plot(time, Uu, time, Uv, time, Uw); grid on
legend('uu','uv','uw')




