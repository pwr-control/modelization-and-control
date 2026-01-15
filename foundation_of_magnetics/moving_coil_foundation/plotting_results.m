close all
clc


figure(1); 
plot(t,position,'-k','LineWidth',1);
hold on
plot(t,position_ref,'--k','LineWidth',1);
hold on
plot(t,fl,'-k','LineWidth',2);
hold off
title('Tracking performance','Interpreter','latex');
legend('$x$','$x^{ref}$','$f_l$','Interpreter','latex','Location','northwest');
set(gca,'xlim',[t(1),t(end)]);
get(gca,'fontname');  % shows you what you are using.
set(gca,'fontname','Times New Roman');  % Set it to times
xlabel('$t/s$','Interpreter','latex');
ylabel({'$x/m$';'$f/N$'},'Interpreter','latex','Rotation',90);
grid on
print -deps2 track_x_3

figure(2); 
subplot 311
plot(t,state(:,1),'-k','LineWidth',1.8);
hold on
plot(t,state_hat(:,1),'--k','LineWidth',1.8);
hold off
title('Estimation of the posisiton','Interpreter','latex');
legend('$x$','$\hat{x}$','Interpreter','latex','Location','northwest');
set(gca,'xlim',[t(1),t(end)]);
get(gca,'fontname');  % shows you what you are using.
set(gca,'fontname','Times New Roman');  % Set it to times
xlabel('$t/s$','Interpreter','latex');
ylabel('$x/m$','Interpreter','latex','Rotation',90);
grid on
subplot 312
plot(t,state(:,2),'-k','LineWidth',1);
hold on
plot(t,state_hat(:,2),'--k','LineWidth',1);
hold off
title('Estimation of the speed','Interpreter','latex');
legend('$v$','$\hat{v}$','Interpreter','latex','Location','northwest');
set(gca,'xlim',[t(1),t(end)]);
get(gca,'fontname');  % shows you what you are using.
set(gca,'fontname','Times New Roman');  % Set it to times
xlabel('$t/s$','Interpreter','latex');
ylabel('$v/m/s$','Interpreter','latex','Rotation',90);
grid on
subplot 313
plot(t,state(:,3),'-k','LineWidth',1);
hold on
plot(t,state_hat(:,3),'--k','LineWidth',1);
hold off
title('Estimation of the current','Interpreter','latex');
legend('$i$','$\hat{i}$','Interpreter','latex','Location','northwest');
set(gca,'xlim',[t(1),t(end)]);
get(gca,'fontname');  % shows you what you are using.
set(gca,'fontname','Times New Roman');  % Set it to times
xlabel('$t/s$','Interpreter','latex');
ylabel('$i/A$','Interpreter','latex','Rotation',90);
grid on
print -deps2 observer_3