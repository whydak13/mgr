%% Params of model. In SI unit unless stated otherwise 
dt=0.005;% 1/fs of discrete model
H=0.22; % total height of the robot
D=0.083; %total length of the robot
m_r=0.7; % robot mass
m_w=0.02267960; % mass of the wheels
d_w=0.089; % wheel diameter
com_h=0.075; % distance from ground to center of the mass
i_r=m_r*(com_h-d_w/2)^2; % robot inertia [gcm^2]
i_w=(m_w*(d_w/2)^2)/2; % wheel intertia
g=9.81;
bet_f=0.05; % friction 
bet_h=0.05;
R=d_w/2;
L=com_h-R;
a11=i_w +(m_w+m_r)*R^2;
a12=m_r*R*L*1; % 1  instead of cos(theta)
a21=m_r*R*L*0; % 0 instead of sin(theta)
a22=i_r+m_r*L^2; 

b21=-m_r*R*L*0; % 0 instead of sin(theta*dtheta)
c2=-m_r*g*L*0; % 0 instead of sin(theta);
d1=(a11-i_w);
g1=-b21/d1;
g2=a12/d1;
g3=(-a21-i_w)/a22;
bet=bet_f; %%%? Sprawdz
h1=bet/(1+g1*g2);
h2=c2/(1+g1*g2);
h3=(g1*g3-bet)/(1+g1*g2);


A=[    0    1        0           0;
       0    -g2*h1   g2*h3   g1-g2*h3;
       0    0        0           1;
       0    h1      -h2          h3];
   