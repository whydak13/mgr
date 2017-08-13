%% robot visualisation script


close all
params;
fig_h=figure(1);
bd = hgtransform;
wh = hgtransform;
body_h=rectangle('Position',[-D/2 0 D H],'Curvature',0.02,'LineWidth',3,'FaceColor',[0.8 .8 .8],'Parent',bd)
hold on
bd.Matrix = makehgtform('zrotate',pi/3)
wheel_h=rectangle('Position',[-d_w/2 -d_w/2 d_w d_w],'Curvature',[1 1],'LineWidth',3,'FaceColor',[0.6 .6 .6],'LineStyle','--','Parent',wh)
grid on
axis( [-0.2 0.2 0 0.4])
% bd.Matrix = makehgtform('translate',[0 d_w/2 0])
% wh.Matrix = makehgtform('translate',[0 d_w/2 0])
%         refreshdata
%         drawnow
xlabel('distance [m]')
step=32;
t=sim_out.time;
theta=sim_out.signals(1).values(:,1)./57.295779513;
phi=sim_out.signals(2).values(:,1)./57.295779513;
    for i=step+1:step:length(t)
    %     bd.Matrix = makehgtform('zrotate',sinus(i)/3)
    
    x_cord=phi(i)*2*(d_w/2);
    y_cord=H+d_w/2+0.1;
        bd.Matrix = makehgtform('translate',[x_cord d_w/2 0],'zrotate',-theta(i));
    %     wh.Matrix = makehgtform('zrotate',-cosinus(i)*6)
        wh.Matrix = makehgtform('translate',[x_cord d_w/2 0],'zrotate',-phi(i));
        axis( [x_cord-0.2 x_cord+0.2 0  0.4]);
       title( strcat('Time: ',num2str(t(i))));
       
        drawnow
         pause((1*t(i)-t(i-step))/250);
%          pause(0.02);

    end

