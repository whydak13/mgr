figure(1)
h = plot([0.5  2.1  2.1  0.5  0.5], [0.5  0.5  1.5  1.5  0.5]);
% h=rectangle('Position',[-D/2 d_w/2 D H],'Curvature',0.02,'LineWidth',3,'FaceColor',[0.8 .8 .8])

axis([0  4  0  2])
axis equal
for k1 = 1:24
    rotate(h, [0 0 1], 0.3*k1)
    refreshdata
    drawnow
end