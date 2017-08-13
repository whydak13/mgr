params;
x_0=[0.01;0.01;0;0];
E=[i_w+(m_w+m_r)*R^2 m_r*R*L
    m_r*R*L i_r+m_r*L^2];
G=[0
    -m_r*g*L];
H=[1 ;-1];
F=[bet_h+bet_f -bet_h; -bet_h bet_h];
Ac=[ 0       0          1     0
     0       0          0     1
    [0;0] -(E^-1)*G    -(E^-1)*F  ];
Bc=[0; 0; -((E^-1)*H)];%Bc=[0; 0; -(E^-1)*H];

% Cc=[R 0 0 0
%     0 1 0 0];

Cc=[0 0 R 0
    0 1 0 0];
% Cc=[R 0 0 0;
%     0 1 0 0;
%     0 0 R 0;
%     0 0 0 1];
% Cc=[0 1 0 0];

sys1=ss(Ac,Bc,Cc,0);
sys_d=c2d(sys1,dt);

Ad=sys_d.A
Bd=sys_d.B
Cd=sys_d.C;