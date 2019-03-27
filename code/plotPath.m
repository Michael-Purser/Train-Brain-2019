function plotPath(k,Arm,X,h1,b1,L1,h2,b2,L2,h3,b3,L3,wg,wo,wl)

xmin = min(min(Arm(1,:,:)));
xmax = max(max(Arm(1,:,:)));
ymin = min(min(Arm(2,:,:)));
ymax = max(max(Arm(2,:,:)));
zmin = min(min(Arm(3,:,:)));
zmax = max(max(Arm(3,:,:)));

figure;

% ARM:
arm = Arm(:,:,k);
plot3(arm(1,1:end-1),arm(2,1:end-1),arm(3,1:end-1),'ko','Linewidth',2); hold all;
plot3(squeeze(Arm(1,end,1:k)),squeeze(Arm(2,end,1:k)),squeeze(Arm(3,end,1:k)),'r','Linewidth',2);
set(gca, 'visible', 'off');

plot3(xmin,arm(2,4),zmin,'ro','Linewidth',2);
plot3(arm(1,4),ymin,zmin,'ro','Linewidth',2);
plot3(arm(1,4),arm(2,4),zmin,'ro','Linewidth',2);
plot3(arm(1,4),ymax,arm(3,4),'ro','Linewidth',2);
plot3([arm(1,4) arm(1,4)],[arm(2,4) arm(2,4)],[arm(3,4) zmin],'r--');
plot3([arm(1,4) arm(1,4)],[arm(2,4) ymax],[arm(3,4) arm(3,4)],'r--');
plot3([xmin arm(1,4)],[arm(2,4) arm(2,4)],[zmin zmin],'r--');
plot3([arm(1,4) arm(1,4)],[arm(2,4) ymin],[zmin zmin],'r--');

plot3([xmin xmax],[0 0],[-0.5 -0.5],'k');

w1 = b1/2;
l1 = L1/2;
d1 = h1/2;
verts = [-w1 -l1 -d1; ...
          w1 -l1 -d1; ...
         -w1  l1 -d1; ...
          w1  l1 -d1; ...
         -w1 -l1  d1; ...
          w1 -l1  d1; ...
         -w1  l1  d1; ...
          w1  l1  d1];
faces = [3 4 8 7; ...
         4 2 6 8; ...
         2 1 5 6; ...
         1 3 7 5; ...
         7 8 6 5; ...
         1 2 4 3];
TF1 = hgtransform;     
patch('Vertices',verts,'Faces',faces,'FaceColor',[255 140 0]/255,'Parent',TF1);
TF1.Matrix = makehgtform('translate',[0.5*(arm(1,1)+arm(1,2))...
    0.5*(arm(2,1)+arm(2,2)) 0.5*(arm(3,1)+arm(3,2))],'zrotate',-X(k,3),'xrotate',pi/2-X(k,1));

w2 = b2/2;
l2 = L2/2;
d2 = h2/2;
verts = [-w2 -l2 -d2; ...
          w2 -l2 -d2; ...
         -w2  l2 -d2; ...
          w2  l2 -d2; ...
         -w2 -l2  d2; ...
          w2 -l2  d2; ...
         -w2  l2  d2; ...
          w2  l2  d2];
faces = [3 4 8 7; ...
         4 2 6 8; ...
         2 1 5 6; ...
         1 3 7 5; ...
         7 8 6 5; ...
         1 2 4 3];
TF2 = hgtransform;     
patch('Vertices',verts,'Faces',faces,'FaceColor',[255 140 0]/255,'Parent',TF2);
TF2.Matrix = makehgtform('translate',[0.5*(arm(1,2)+arm(1,3))...
    0.5*(arm(2,2)+arm(2,3)) 0.5*(arm(3,2)+arm(3,3))],'zrotate',-X(k,3),'xrotate',pi/2-X(k,2));

w3 = b3/2;
l3 = L3/2;
d3 = h3/2;
verts = [-w3 -l3 -d3; ...
          w3 -l3 -d3; ...
         -w3  l3 -d3; ...
          w3  l3 -d3; ...
         -w3 -l3  d3; ...
          w3 -l3  d3; ...
         -w3  l3  d3; ...
          w3  l3  d3];
faces = [3 4 8 7; ...
         4 2 6 8; ...
         2 1 5 6; ...
         1 3 7 5; ...
         7 8 6 5; ...
         1 2 4 3];
TF3 = hgtransform;     
patch('Vertices',verts,'Faces',faces,'FaceColor',[255 140 0]/255,'Parent',TF3);
TF3.Matrix = makehgtform('translate',[0.5*(arm(1,3)+arm(1,4))...
    0.5*(arm(2,3)+arm(2,4)) 0.5*(arm(3,3)+arm(3,4))],...
    'zrotate',-pi/2+atan2(arm(2,4)-arm(2,3),arm(1,4)-arm(1,3)));


% WAGON EN DRAAIENDE SCHIJF WAAROP ARM ZIT:

Lw = 8; % length wagon
Ww = 3; % width wagon
Hw = 0.5; % height platform representing wagon
Zw = -0.3; % vertical offset between wagon and platform
verts = [-Lw/2 -Ww/2  Zw; ...
          Lw/2 -Ww/2  Zw; ...
         -Lw/2  Ww/2  Zw; ...
          Lw/2  Ww/2  Zw; ...
         -Lw/2 -Ww/2  Zw-Hw; ...
          Lw/2 -Ww/2  Zw-Hw; ...
         -Lw/2  Ww/2  Zw-Hw; ...
          Lw/2  Ww/2  Zw-Hw];
      
faces = [1 2 4 3];
TFw = hgtransform;     
patch('Vertices',verts,'Faces',faces,'FaceColor',[.75 .75 .75],'Parent',TFw);
TFw.Matrix = makehgtform('translate',[arm(1,1) 0 0]);
      
faces = [3 4 8 7; ...
         4 2 6 8; ...
         2 1 5 6; ...
         1 3 7 5; ...
         7 8 6 5];
TFw = hgtransform;     
patch('Vertices',verts,'Faces',faces,'FaceColor',[.5 .5 .5],'Parent',TFw);
TFw.Matrix = makehgtform('translate',[arm(1,1) 0 0]);

arc = linspace(0,2*pi,100);
R = 1;
fill3(R*cos(arc)+arm(1,1),R*sin(arc),zeros(size(arc)),[255 140 0]/255);
[Xc,Yc,Zc] = cylinder(1,50);
s1 = surf(Xc+arm(1,1),Yc,0.7*(Zc-1)); 
s1.FaceColor = [255 140 0]/255;

plot3([-R*cos(X(k,3)) R*cos(X(k,3))]+arm(1,1), [R*sin(X(k,3)) -R*sin(X(k,3))],[0 0],'k','Linewidth',1.5);


% PLATFORM EN LUIFELS:
[v,f1,f2,f3,f4,fp1,fp2,fp3] = makeEnv(wl);

TFen = hgtransform;
TFen.Matrix = makehgtform('translate',[0 wg+wo 0]);

patch('Vertices',v,'Faces',f1,'FaceColor',[.75 .75 .75],'Parent',TFen);

patch('Vertices',v,'Faces',f2,'FaceColor',[.5 .5 .5],'Parent',TFen);

patch('Vertices',v,'Faces',f3,'FaceColor',[.75 .75 .75],'Parent',TFen);

patch('Vertices',v,'Faces',f4,'FaceColor',[.5 .5 .5],'Parent',TFen);


patch('Vertices',v,'Faces',fp1,'FaceColor',[.75 .75 .75],'Parent',TFen);

patch('Vertices',v,'Faces',fp2,'FaceColor',[.5 .5 .5],'Parent',TFen);

patch('Vertices',v,'Faces',fp3,'FaceColor',[.5 .5 .5],'Parent',TFen);


% PALEN:
[Xc,Yc,Zc] = cylinder(0.3,10);
s1 = surf(Xc+3,Yc+5,Zc*6); 
s1.FaceColor = [0.5 0.5 0.5];
% s1.EdgeColor = 'none';

s2 = surf(Xc+11,Yc+5,Zc*3); 
s2.FaceColor = [0.5 0.5 0.5];
% s2.EdgeColor = 'none';


% BOVENLEIDING:
x = linspace(0,14,300);
fb = @(x) (0.3/49)*(x-7).^2 + 3.7;

plot3(x,zeros(size(x)),fb(x),'b','Linewidth',2);
plot3(x,zeros(size(x)),fb(x)+0.4,'b','Linewidth',2);

axis equal;
axis([-4 18 -1.5 10 -1 7]);

view(-40,20);
% view(-90,0);

end