clear
close all
init_lin;

Dp=0;
Np=15;
Nc=3;
xm=[0;0.1;0;0];% initial state

[Phi,Phi_Phi,Phi_F,Phi_R,A_e, B_e,C_e,F]=mpc_gain(Ad,Bd,Cd,Nc,Np);
[m1,n1]=size(C_e);
[n,n_in]=size(B_e);



%% SIM PART
Xf=zeros(n,1);
N_sim=4;

r=zeros(N_sim,m1);
% r(40:60,1)=0.01*ones(21,1);
% r(70:90,1)=-0.02*ones(21,1);
u=0; % u(k-1) =0
y=0;

u1=(1:N_sim).*0;
y1=zeros(N_sim,m1);

%% Contraits
max_u=15;
max_u_delta=0.1;
A_cons = [+toeplitz(ones(1,Nc),[1 zeros(1,Nc-1)])
         -toeplitz(ones(1,Nc),[1 zeros(1,Nc-1)])
         eye(Nc);-eye(Nc)
];
     
% A_cons=  [eye(Nc);-eye(Nc)];


%% Simulation
for kk=1:N_sim;

B_cons = get_b_constraints(Nc, max_u_delta, max_u, u1(kk) );
% [max_u_delta * ones(Nc*2,1)- u1(kk)
%           max_u * ones(Nc*2,1)+ u1(kk)];
[ f ] =get_f( Np, r(kk,:), F, Xf, Phi );
%     [ f ] =get_f( Np, [0.1 0], F, Xf, Phi );
      DeltaU=QPhild2(Phi_Phi,f,A_cons,B_cons)
%      DeltaU=quadprog(Phi_Phi, f,A_cons, B_cons, [], [], [], [], [], optimset('Algorithm', 'interior-point-convex'));
%    DeltaU=quadprog(Phi_Phi, f,[], [], [], [], [], [], [], optimset('Algorithm', 'interior-point-convex'));
  
   deltau=DeltaU(1,1);
   u=u+deltau;
   u1(kk)=sign(u)*min(abs(u),max_u);
   y1(kk,:)=y;
   xm_old=xm;
   xm=Ad*xm+Bd*u;
   y=Cd*xm;
   Xf=[xm-xm_old;y];
end

%% Display
k=(0:(N_sim-1))*dt;
figure
subplot(311)
plot(k,y1(:,1))
hold on
grid on
plot(k,r(:,1))
legend('Speed','Speed sp')
subplot(312)
plot(k,y1(:,2))
hold on
grid on
plot(k,r(:,2))
xlabel('Time[s]')
legend('Angle','Angle sp')
subplot(313)
plot(k,u1)
grid on
xlabel('Time[s]')
legend('Control')