clear all
close all
clc

Ts = 1/1e3;
s=tf('s');
z=tf('z',Ts);

b = 20e-3;
h = 200e-3;
m = 30e-3;
Bm = 1;
L = 1e-3;
R = 0.1;

A = [0 1 0; ...
    0 -b/m Bm*h/m; ...
    0 -Bm*h/L -R/L];
B = [0 0 1/L]';
E = [0 -1/m 0]';
C = [1 0 0; 0 0 1];

Ale = [0 1 0 0; ...
    0 -b/m Bm*h/m -1/m; ...
    0 -Bm*h/L -R/L 0; ...
    0 -b/m Bm*h/m -1/m;];
Ble = [0 0 1/L 0]';
Cle = [1 0 0 0; 0 0 1 0];

% Ad = eye(3) + A*Ts+ A^2*Ts^2/2;
% Bd = B*Ts + A*B*Ts^2/2 + A^2*B*Ts^3/factorial(3);
Ad = eye(3) + A*Ts;
Bd = B*Ts;

% Aled = eye(4) + Ale*Ts+ Ale^2*Ts^2/2;
% Bled = Ble*Ts + Ale*Ble*Ts^2/2 + Ale^2*Ble*Ts^3/factorial(3);
Aled = eye(4) + Ale*Ts;
Bled = Ble*Ts;

rank(obsv(A,C))
rank(ctrb(A,B))
rank(obsv(Aled,Cle))
rank(ctrb(Ad,Bd))

poles_so = [-45,-50,-100]*2*pi;

% 45, 50, 100, 200
% My current poles for the observer are at -2*pi*[45, 50, 100, 200] and
% for the controller at -2*pi*[4.5, 5.0, 5.5, 6.0].
    
    
% poles_so_le = [-45,-50,-100 -80]*2*pi;
poles_so_le = -[45, 50, 100, 120]*2*pi;
% poles_sf = poles_so_le/5;
% poles_sf = -2*pi*[4.5, 5.0, 5.5, 6.0];
poles_sf = -2*pi*[4.5, 5.0, 5.5, 6.0]*3/2;

poles_sod = exp(Ts*poles_so);

poles_so_led = exp(Ts*poles_so_le);

Ld = place(Ad',C',poles_sod)'

Lled = place(Aled',Cle',poles_so_led)'

poles_sfd = exp(Ts*poles_sf);

% Kd = place(Ad,Bd,poles_sfd)

% poles_sfe = [poles_sf -2.5*2*pi];
poles_sfe = poles_sf; 
poles_sfed = exp(Ts*poles_sfe);

Ade = [Ad Bd; [0 0 0] 0];
Bde = [[0 0 0]';1];
Ke_hat = acker(Ade,Bde,poles_sfed)
Ked = (Ke_hat+[0 0 0 1])/([Ad-eye(3) Bd; C(1,:)*Ad C(1,:)*Bd])

Tc=1e-5;
simlength = 1;
Nc=floor(simlength/Tc);

open_system('mc');
