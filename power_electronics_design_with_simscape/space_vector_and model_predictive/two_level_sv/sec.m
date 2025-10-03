clear all
close all
clc
beep off

ts = 1e-4;
T = 20e-3;
time = [0:ts:2*T-ts];
theta = 2*pi/T*time;
u1 = cos(theta);
u2 = cos(theta - 2*pi/3);
u3 = cos(theta + 2*pi/3);

ua = 2/3*(u1 - 1/2*u2 - 1/2*u3);
ub = 2/3*(sqrt(3)/2*u2 - sqrt(3)/2*u3);

a = 2*pi/3;
q1 = a;
q2 = 2*a;

x1 = cos(q1);
x2 = cos(q2);
y1 = sin(q1);
y2 = sin(q2);

sec_x = (sign(u1) + x1*sign(u2) + x2*sign(u3));
sec_y = (y1*sign(u2) + y2*sign(u3));
sector = (sign(u1)>0)*1 + (sign(u2)>0)*2 + (sign(u3)>0)*4; 


figure; 
plot(sec_x, sec_y,'-r'); 
hold on
plot(ua, ub,'-k'); 
hold off
axis equal;
grid on

% figure; 
% plot(time, ua, time, ub, ...
%     time, sector); 
% legend('ua','ub','sector')
% grid on

figure; 
plot(time, u1, time, u2, time, u3, ...
    time, sector); 
legend('u1','u2','u3','sx','sy')
grid on
% 
% figure; 
% plot(time, ua, time, ub, ...
%     time, sec_x, time, sec_y); 
% legend('ua','ub','sx','sy')
% grid on



