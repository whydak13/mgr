%% robot visualisation script

t=0:0.02:2;
sinus=sin(2*pi*t);
cosinus=cos(2*pi*t);
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
while(1)
    for i=1:length(sinus)
    %     bd.Matrix = makehgtform('zrotate',sinus(i)/3)
    x_cord=cosinus(i)*2*(d_w/2);
    y_cord=H+d_w/2+0.1;
        bd.Matrix = makehgtform('translate',[x_cord d_w/2 0],'zrotate',sinus(i)/6);
    %     wh.Matrix = makehgtform('zrotate',-cosinus(i)*6)
        wh.Matrix = makehgtform('translate',[x_cord d_w/2 0],'zrotate',-cosinus(i)*2);
        axis( [x_cord-0.2 x_cord+0.2 0  y_cord]);
       title( strcat('Time: ',num2str(t(i))));
       
        drawnow
         pause(0.02);

    end
end
