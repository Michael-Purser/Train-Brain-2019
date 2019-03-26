clear;
clc;

% Data
wl = 7.76;
wg = 0.5;
wo = 1.5;
hl = 3.9;
ho = 3.5;

sigb = 218e6;
tau = 126e6;
E = 210e9;
g = 9.81;
% mA
mB = 0;
mC = 0;
mL1 = 9.08;
mL2 = 9.08;
mL3 = 9.08;
h1 = 100e-3;
h2 = 100e-3;
h3 = 100e-3;
b1 = 50e-3;
b2 = 50e-3;
b3 = 50e-3;
t1 = 2*6.3e-3;
t2 = 2*6.3e-3;
t3 = 2*6.3e-3;


% Geometry

Cdesign = [0;wg+wo;hl+0.5];

L = sqrt((Cdesign(2))^2+(Cdesign(3))^2);
L1 = 1.05*0.5*L;
L2 = L1;
L3 = 0.5*wl;

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
Cstart = [0;1;2];
Cend   = [vx*T;1;2];

% make goals:
Cgoals = [linspace(Cstart(1),Cend(1),N);linspace(Cstart(2),Cend(2),N);...
    linspace(Cstart(3),Cend(3),N)];
% Cgoals = [linspace(Cstart(1),Cend(1),N);linspace(Cstart(2),Cend(2),N);...
%     Cstart(3)+0.2*sin(10*linspace(Cstart(1),Cend(1),N))];
% Cgoals = [Cstart(1)*ones(1,N);linspace(Cstart(2),Cend(2),N);...
%     Cstart(3)+0.2*sin(10*linspace(Cstart(1),Cend(1),N))];

for k = 1:N
    
Cgoal = Cgoals(:,k);
A = [vx*k*T/(N-1);0;0];

l = sqrt((Cgoal(2)-A(2))^2 + (Cgoal(1)-A(1))^2);
if l == 0
    psi = 0;
else
    psi = -asin((Cgoal(2)-A(2))/l);
end

if k==1
    x0 = [pi/2,0];
else
    x0 = 0.6*X(k-1,1:2);
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
%     plot3(arm(1,:),arm(2,:),arm(3,:),'bo-','Linewidth',2); hold all;
%     plot3(squeeze(Arm(1,end,1:k)),squeeze(Arm(2,end,1:k)),squeeze(Arm(3,end,1:k)),'r','Linewidth',2);
% 
%     plot3(xmin,arm(2,4),zmin,'ko','Linewidth',2);
%     plot3(arm(1,4),ymin,zmin,'ko','Linewidth',2);
%     plot3(arm(1,4),arm(2,4),zmin,'ko','Linewidth',2);
%     plot3(arm(1,4),ymax,arm(3,4),'ko','Linewidth',2);
%     plot3([arm(1,4) arm(1,4)],[arm(2,4) arm(2,4)],[arm(3,4) zmin],'k--');
%     plot3([arm(1,4) arm(1,4)],[arm(2,4) ymax],[arm(3,4) arm(3,4)],'k--');
%     plot3([xmin arm(1,4)],[arm(2,4) arm(2,4)],[zmin zmin],'k--');
%     plot3([arm(1,4) arm(1,4)],[arm(2,4) ymin],[zmin zmin],'k--');
%     
%     plot3([xmin xmax],[0 0],[0 0],'k');
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
% name = 'test_simpleFollowing.avi';
% v = VideoWriter(name);
% v.Quality = 100;
% v.FrameRate = 10;
% open(v);
% writeVideo(v,F);
% close(v);

figure;
arm = Arm(:,:,k);
plot3(arm(1,:),arm(2,:),arm(3,:),'bo-','Linewidth',2); hold all;
plot3(squeeze(Arm(1,end,1:k)),squeeze(Arm(2,end,1:k)),squeeze(Arm(3,end,1:k)),'r','Linewidth',2);

plot3(xmin,arm(2,4),zmin,'ko','Linewidth',2);
plot3(arm(1,4),ymin,zmin,'ko','Linewidth',2);
plot3(arm(1,4),arm(2,4),zmin,'ko','Linewidth',2);
plot3(arm(1,4),ymax,arm(3,4),'ko','Linewidth',2);
plot3([arm(1,4) arm(1,4)],[arm(2,4) arm(2,4)],[arm(3,4) zmin],'k--');
plot3([arm(1,4) arm(1,4)],[arm(2,4) ymax],[arm(3,4) arm(3,4)],'k--');
plot3([xmin arm(1,4)],[arm(2,4) arm(2,4)],[zmin zmin],'k--');
plot3([arm(1,4) arm(1,4)],[arm(2,4) ymin],[zmin zmin],'k--');

plot3([xmin xmax],[0 0],[0 0],'k');


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
fprintf('\n');
fprintf('Link 2: \n');
fprintf('\t sigb2 = %f [MPa] \n',sigb2*1e-6);
fprintf('\t tau2 = %f [MPa] \n',tau2*1e-6);
fprintf('\n');
fprintf('Link 1: \n');
fprintf('\t sigb1 = %f [MPa] \n',sigb1*1e-6);
fprintf('\t tau1 = %f [MPa] \n',tau1*1e-6);