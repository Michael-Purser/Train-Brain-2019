clear;
clc;

% Data
wl = 7.76;
wg = 0.5;
wo = 1.5;
hl = 3.9;
ho = 3.5;

sigb = 218e6; % maximaal toelaatbare buigspanning voor staal S235JR
tau = 126e6;  % maximaal toelaatbare schuifspanning voor staal S235JR  
E = 210e9;    % E-modulus staal
g = 9.81;     % gravity
% mA
mB = 500;    % massa motor in B
mC = 500;    % massa motor in C
mL1 = 9.08;  % massa link 1 per lengte-eenheid
mL2 = 9.08;  % massa link 2 per lengte-eenheid
mL3 = 9.08;  % massa link 3 per lengte-eenheid
h1 = 200e-3; % hoogte profiel link 1
h2 = 200e-3;
h3 = 100e-3;
b1 = 100e-3; % breedte profiel link 1
b2 = 100e-3;
b3 = 50e-3;
t1 = 2*6.3e-3; % totale dikte profiel link 1
t2 = 2*6.3e-3;
t3 = 2*6.3e-3;


% Geometry

Cdesign = [0;wg+wo;hl+0.7]; % Verste punt dat de arm zeker moet kunnen halen

L = sqrt((Cdesign(2))^2+(Cdesign(3))^2);
L1 = 1.05*0.5*L;  % Marge van 5% genomen zodat arm in verste punt niet in sigulaire opstelling is.
L2 = L1;
L3 = 0.5*wl;  % Helft van luifelbreedte wl.

fprintf('\n');
fprintf('Geometry: \n');
fprintf('\t L1 = %f [m] \n',L1);
fprintf('\t L2 = %f [m] \n',L2);
fprintf('\t L3 = %f [m] \n',L3);


% Make movie of moving arm:

N = 50;
X = zeros(N,3);
Arm = zeros(3,4,N);
vx     = 0.1; % m/s
T      = 10;
Cstart = [0;L1+L2;0];
Cend   = [0;L1+L2;0];
tol    = 1e-5;

% make goals:
Cgoals = makeGoals(Cstart,Cend,N,1);

for k = 1:N
    
Cgoal = Cgoals(:,k);
A = [0*vx*(k-1)*T/(N-1);0;0];

l = sqrt((Cgoal(2)-A(2))^2 + (Cgoal(1)-A(1))^2);
if l < tol
    psi = 0;
else
    psi = acos((Cgoal(2)-A(2))/l);
    if Cgoal(1)<A(1)
        psi = -psi;
    end
end

if k==1
    x0 = [pi/2,0];
else
    x0 = 0.9*X(k-1,1:2);
end
options = optimset('Display','off');
[x,~,exitflag] = fsolve(@(x) armGeom(x,A,Cgoal,psi,L1,L2),x0,options);
if (exitflag ~= 1)
    error('The fsolve exit flag was not 1, probably no convergence!');
end

X(k,:) = [x psi];

B = A + [(L1*sin(x(1)))*sin(psi);(L1*sin(x(1)))*cos(psi);L1*cos(x(1))];
C = B + [(L2*sin(x(2)))*sin(psi);(L2*sin(x(2)))*cos(psi);L2*cos(x(2))];
D = C + [0;L3;0];
Arm(:,:,k) = [A B C D];

end

xmin = min(min(Arm(1,:,:)));
xmax = max(max(Arm(1,:,:)));
ymin = min(min(Arm(2,:,:)));
ymax = max(max(Arm(2,:,:)));
zmin = min(min(Arm(3,:,:)));
zmax = max(max(Arm(3,:,:)));
 
% for k=1:N
%     arm = Arm(:,:,k);
%     plot3(arm(1,:),arm(2,:),arm(3,:),'ko','Linewidth',2); hold all;
%     plot3(squeeze(Arm(1,end,1:k)),squeeze(Arm(2,end,1:k)),squeeze(Arm(3,end,1:k)),'r','Linewidth',2);
% 
%     plot3(xmin,arm(2,4),zmin,'ro','Linewidth',2);
%     plot3(arm(1,4),ymin,zmin,'ro','Linewidth',2);
%     plot3(arm(1,4),arm(2,4),zmin,'ro','Linewidth',2);
%     plot3(arm(1,4),ymax,arm(3,4),'ro','Linewidth',2);
%     plot3([arm(1,4) arm(1,4)],[arm(2,4) arm(2,4)],[arm(3,4) zmin],'r--');
%     plot3([arm(1,4) arm(1,4)],[arm(2,4) ymax],[arm(3,4) arm(3,4)],'r--');
%     plot3([xmin arm(1,4)],[arm(2,4) arm(2,4)],[zmin zmin],'r--');
%     plot3([arm(1,4) arm(1,4)],[arm(2,4) ymin],[zmin zmin],'r--');
%     
%     plot3([xmin xmax],[0 0],[0 0],'k');
%     
%     w1 = h1/2;
%     l1 = L1/2;
%     d1 = b1/2;
%     verts = [-w1 -l1 -d1; ...
%               w1 -l1 -d1; ...
%              -w1  l1 -d1; ...
%               w1  l1 -d1; ...
%              -w1 -l1  d1; ...
%               w1 -l1  d1; ...
%              -w1  l1  d1; ...
%               w1  l1  d1];
%     faces = [3 4 8 7; ...
%              4 2 6 8; ...
%              2 1 5 6; ...
%              1 3 7 5; ...
%              7 8 6 5; ...
%              1 2 4 3];
%     TF1 = hgtransform;     
%     patch('Vertices',verts,'Faces',faces,'FaceColor',[.75 .75 .75],'Parent',TF1);
%     TF1.Matrix = makehgtform('translate',[0.5*(arm(1,1)+arm(1,2))...
%         0.5*(arm(2,1)+arm(2,2)) 0.5*(arm(3,1)+arm(3,2))],'zrotate',-X(k,3),'xrotate',pi/2-X(k,1));
% 
%     w2 = h2/2;
%     l2 = L2/2;
%     d2 = b2/2;
%     verts = [-w2 -l2 -d2; ...
%               w2 -l2 -d2; ...
%              -w2  l2 -d2; ...
%               w2  l2 -d2; ...
%              -w2 -l2  d2; ...
%               w2 -l2  d2; ...
%              -w2  l2  d2; ...
%               w2  l2  d2];
%     faces = [3 4 8 7; ...
%              4 2 6 8; ...
%              2 1 5 6; ...
%              1 3 7 5; ...
%              7 8 6 5; ...
%              1 2 4 3];
%     TF2 = hgtransform;     
%     patch('Vertices',verts,'Faces',faces,'FaceColor',[.75 .75 .75],'Parent',TF2);
%     TF2.Matrix = makehgtform('translate',[0.5*(arm(1,2)+arm(1,3))...
%         0.5*(arm(2,2)+arm(2,3)) 0.5*(arm(3,2)+arm(3,3))],'zrotate',-X(k,3),'xrotate',pi/2-X(k,2));
% 
%     w3 = h3/2;
%     l3 = L3/2;
%     d3 = b3/2;
%     verts = [-w3 -l3 -d3; ...
%               w3 -l3 -d3; ...
%              -w3  l3 -d3; ...
%               w3  l3 -d3; ...
%              -w3 -l3  d3; ...
%               w3 -l3  d3; ...
%              -w3  l3  d3; ...
%               w3  l3  d3];
%     faces = [3 4 8 7; ...
%              4 2 6 8; ...
%              2 1 5 6; ...
%              1 3 7 5; ...
%              7 8 6 5; ...
%              1 2 4 3];
%     TF3 = hgtransform;     
%     patch('Vertices',verts,'Faces',faces,'FaceColor',[.75 .75 .75],'Parent',TF3);
%     TF3.Matrix = makehgtform('translate',[arm(1,3) arm(2,3)+l3 arm(3,3)]);
% 
%     
%     axis equal;
%     box on;
%     grid on;
%     axis([xmin xmax ymin ymax zmin zmax]);
%     drawnow;
%     set(gcf,'Position',[1 1 450*2 570*2]);
%     F(k) = getframe(gcf);
%     close(gcf)
% end
% 
% name = 'TEST.avi';
% v = VideoWriter(name);
% v.Quality = 100;
% v.FrameRate = 10;
% open(v);
% writeVideo(v,F);
% close(v);

plotArm(k,Arm,X,h1,b1,L1,h2,b2,L2,h3,b3,L3,wg,wo,wl)


% Strength and stiffness:

factor = 1.5;

% Link3:

[I3,H3] = momentsArea(h3,b3,t3);
ymax3 = h3/2;
G3 = -mL3*L3*g;
R3 = -G3;
M3 = 0.5*L3*G3;
Vmax3 = factor*abs(R3);
Mmax3 = factor*M3;
sigb3 = abs(Mmax3)*ymax3/I3;
tau3 = Vmax3*H3/(I3*t3);

f3  = 1/3*abs(G3)*L3^3/(E*I3);
phi3 = 0.5*abs(G3)*(L3)^2/(E*I3);
f3 = f3+0.5*L*sin(phi3);

% Link2:

[I2,H2] = momentsArea(h2,b2,t2);
ymax2 = h2/2;
G2 = -mL2*L2*g;
GC = -mC*g;
R2 = -GC + R3 - G2;
M2 = 0.5*L2*G2 + L2*(GC-R3)+M3;
Vmax2 = factor*abs(R2);
Mmax2 = factor*M2;
sigb2 = abs(Mmax2)*ymax2/I2;
tau2 = Vmax2*H2/(I2*t2);

% Link2:

[I1,H1] = momentsArea(h1,b1,t1);
ymax1 = h1/2;
G1 = -mL1*L1*g;
GB = -mB*g;
R1 = -GB + R2 - G1;
M1 = 0.5*L1*G1 + L1*(GB-R2)+M2;
Vmax1 = factor*abs(R1);
Mmax1 = factor*M1;
sigb1 = abs(Mmax1)*ymax1/I1;
tau1 = Vmax1*H1/(I1*t1);

% f2  = 1/3*abs(G3)*L3^3/(E*I3);
% phi2 = 0.5*abs(G3)*(L3)^2/(E*I3);
% f2 = f2+0.5*L*sin(phi3);

fprintf('\n');
fprintf('Strength limits: \n');
fprintf('\t sigb = %f [MPa] \n',sigb*1e-6);
fprintf('\t tau = %f [MPa] \n',tau*1e-6);
fprintf('\n');
fprintf('Link 3: \n');
fprintf('\t sigb3 = %f [MPa] \n',sigb3*1e-6);
fprintf('\t tau3 = %f [MPa] \n',tau3*1e-6);
fprintf('\t f3 = %f [m] \n',f3);
fprintf('\t M3 = %f [m] \n',M3);
fprintf('\n');
fprintf('Link 2: \n');
fprintf('\t sigb2 = %f [MPa] \n',sigb2*1e-6);
fprintf('\t tau2 = %f [MPa] \n',tau2*1e-6);
fprintf('\t M2 = %f [m] \n',M2);
fprintf('\n');
fprintf('Link 1: \n');
fprintf('\t sigb1 = %f [MPa] \n',sigb1*1e-6);
fprintf('\t tau1 = %f [MPa] \n',tau1*1e-6);
fprintf('\t M1 = %f [m] \n',M1);