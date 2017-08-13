close all
t=sim_out_nonlin.time;
theta_nl=sim_out_nonlin.signals(1).values(:,1);
dtheta_nl=sim_out_nonlin.signals(1).values(:,2);
phi_nl=sim_out_nonlin.signals(2).values(:,1);
dphi_nl=sim_out_nonlin.signals(2).values(:,2);

theta_l=sim_out_lin.signals(1).values(:,1);
dtheta_l=sim_out_lin.signals(1).values(:,2);
phi_l=sim_out_lin.signals(2).values(:,1);
dphi_l=sim_out_lin.signals(2).values(:,2);

subplot(2,2,1)
title('Theta(body)')
hold on; grid on;
plot(t,theta_l,'r' )
hold on; grid on;
plot(t,theta_nl,'g' )
legend('Lin model','Non lin model')

subplot(2,2,2)
title('Phi(wheel)')
hold on; grid on;
plot(t,phi_l,'r' )
hold on; grid on;
plot(t,phi_nl,'g' )
legend('Lin model','Non lin model')

subplot(2,2,3)
title('dTheta(body)')
hold on; grid on;
plot(t,dtheta_l,'r' )
hold on; grid on;
plot(t,dtheta_nl,'g' )
legend('Lin model','Non lin model')

subplot(2,2,4)
title('dPhi(wheel)')
hold on; grid on;
plot(t,dphi_l,'r' )

plot(t,dphi_nl,'g' )
legend('Lin model','Non lin model')