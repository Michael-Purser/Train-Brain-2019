clear;
close all;
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
h1 = 200e-3;
h2 = 200e-3;
h3 = 100e-3;
b1 = 100e-3;
b2 = 100e-3;
b3 = 50e-3;
t1 = 2*6.3e-3;
t2 = 2*6.3e-3;
t3 = 2*6.3e-3;


% Geometry

Cdesign = [0;wg+wo;hl+0.7];

L = sqrt((Cdesign(2))^2+(Cdesign(3))^2);
L1 = 1.05*0.5*L;
L2 = L1;
L3 = 0.5*wl;

fprintf('\n');
fprintf('Geometry: \n');
fprintf('\t L1 = %f [m] \n',L1);
fprintf('\t L2 = %f [m] \n',L2);
fprintf('\t L3 = %f [m] \n',L3);


% Get data of trajectory:
load('xzycoor.mat');
Cgoals = xzycoor(1:1400,:)';
Cgoals(3,:) = Cgoals(3,:)-L3;
Cgoals([2 3],:) = Cgoals ([3 2],:);

N = size(Cgoals,2);
N = 1400;
X = zeros(N,3);
Arm = zeros(3,4,N);
T      = (N/size(Cgoals,2))*10;
vx     = (Cgoals(1,N)-Cgoals(1,1))/T; % m/s
Cstart = [0;1;2];
Cend   = [2*vx*T;1;2];
tol    = 1e-5;

% make goals:
%Cgoals = makeGoals(Cstart,Cend,N,5);

for k = 1:N
fprintf('Iteration %i \n',k);
Cgoal = Cgoals(:,k);
A = [vx*(k-1)*T/(N-1);0;0];

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
 
for k=1:N
    fprintf('Making frame %i \n',k);
    plotPath(k,Arm,X,h1,b1,L1,h2,b2,L2,h3,b3,L3,wg,wo,wl);
    drawnow;
    pause(0.5);
    set(gcf,'Position',[1 1 500 500]);
%     pause(1);
    F(k) = getframe(gcf);
%     pause(1);
    close(gcf);
end

name = 'TEST3.avi';
v = VideoWriter(name);
v.Quality = 90;
v.FrameRate = 20;
open(v);
writeVideo(v,F);
close(v);

plotPath(k,Arm,X,h1,b1,L1,h2,b2,L2,h3,b3,L3,wg,wo,wl);


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