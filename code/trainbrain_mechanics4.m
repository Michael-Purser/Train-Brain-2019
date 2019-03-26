clear;
close all;
clc;

% Data
wl = 7.76;      % Luifelbreedte
wg = 0.5;       % Verschil halfbreedtes luifel/perron
wo = 1.5;       % Afstand tussen perron rand en midden van rails
hl = 3.9;       % Hoogte bovenkant luifel (vanaf perrron)
ho = 3.5;       % Hoogte onderkant kabels

sigb = 218e6;   % maximaal toelaatbare buigspanning voor staal S235JR
tau = 126e6;    % maximaal toelaatbare schuifspanning voor staal S235JR  
E = 210e9;      % E-modulus staal
g = 9.81;       % gravity
% mA
mB = 50;        % massa motor in B
mC = 50;        % massa motor in C
mL1 = 9.08;     % massa link 1 (per lengte-eenheid)
mL2 = 9.08;     % massa link 2 (per lengte-eenheid)
mL3 = 9.08;     % massa link 3 (per lengte-eenheid)
h1 = 400e-3;    % hoogte profiel link 1
h2 = 400e-3;
h3 = 200e-3;
b1 = 200e-3;    % breedte profiel link 1
b2 = 200e-3;
b3 = 100e-3;
t1 = 2*6.3e-3;  % totale dikte profiel link 1
t2 = 2*6.3e-3;
t3 = 2*6.3e-3;


% Geometrie bepalen:

% Verste punt dat de arm zeker moet kunnen halen:
Cdesign = [0;wg+wo;hl+0.7]; 

L = sqrt((Cdesign(2))^2+(Cdesign(3))^2);
L1 = 1.05*0.5*L;    % 5% marge zodat arm in verste punt niet sigulair is.
L2 = L1;
L3 = 0.5*wl;        % Helft van luifelbreedte wl.

fprintf('\n');
fprintf('Geometry: \n');
fprintf('\t L1 = %f [m] \n',L1);
fprintf('\t L2 = %f [m] \n',L2);
fprintf('\t L3 = %f [m] \n',L3);


% Get data of trajectory:
load('xzycoor.mat');
start                   = [-2 0 0];
Cgoals_main             = xzycoor(1:1400,:)';
Cgoals_main([2 3],:)    = Cgoals_main([3 2],:); % Adapt for ordering
Cgoals_deploy           = trajDeploy([[-0.1 L3 1]+start;Cgoals_main(:,1)'+start]',100);
Cgoals_translate        = [linspace(Cgoals_deploy(1,end);C_main(1,1),100);ones(Cgoals_deploy(2,end),100),...
                                ones(Cgoals_deploy(3,end),100)];
Cgoals                  = [Cgoals_deploy(:,1:end-1) Cgoals_main];

n_deploy                = size(Cgoals_deploy,2)-1;
n_translate             = size(Cgoals_translate,2)-1;
n_main                  = size(Cgoals_main,2);

Cgoals(2,:)             = Cgoals(2,:)-L3;       % Adapt for length of third link

N       = size(Cgoals,2);
X       = zeros(N,3);
Arm     = zeros(3,4,N);
T       = (N/size(Cgoals,2))*10;
vx      = (Cgoals(1,N)-Cgoals(1,1))/T; % m/s
Cstart  = [0;1;2];
Cend    = [2*vx*T;1;2];
tol     = 1e-5;

% make goals:

% deploy
for k = 1:n_deploy
fprintf('Iteration %i \n',k);
Cgoal = Cgoals(:,k);
A = start';

l = sqrt((Cgoal(2)-A(2))^2 + (Cgoal(1)-A(1))^2);
if l < tol
    psi = pi/2;
else
    psi = acos((Cgoal(2)-A(2))/l);
    % sign correction due to cosine two solutions:
    if Cgoal(1)<A(1)
        psi = -psi;
    end
end

if k==1
    x0 = [0,0];
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

% translate
for k = n_deploy+1:(n_deploy+n_translate)
fprintf('Iteration %i \n',k);
Cgoal = Cgoals(:,k);
A = start' + [vx*(k-1-n_deploy)*T/(N-1);0;0];

l = sqrt((Cgoal(2)-A(2))^2 + (Cgoal(1)-A(1))^2);
if l < tol
    psi = pi/2;
else
    psi = acos((Cgoal(2)-A(2))/l);
    % sign correction due to cosine two solutions:
    if Cgoal(1)<A(1)
        psi = -psi;
    end
end

if k==1
    x0 = [0,0];
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

% main cleaning
for k = (n_deploy+n_translate)+1:(n_deploy+n_translate+n_main)
fprintf('Iteration %i \n',k);
Cgoal = Cgoals(:,k);
A = [vx*(k-1-n_deploy)*T/(N-1);0;0];

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
    x0 = [0,pi/2];
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
 

% MAKE MOVIE

% for k=1:N
%     fprintf('Making frame %i \n',k);
%     plotPath(k,Arm,X,h1,b1,L1,h2,b2,L2,h3,b3,L3,wg,wo,wl);
%     drawnow;
%     pause(0.5);
%     set(gcf,'Position',[1 1 500 500]);
% %     pause(1);
%     F(k) = getframe(gcf);
% %     pause(1);
%     close(gcf);
% end
% 
% name = 'MOVIE_SCHUIN.avi';
% v = VideoWriter(name);
% v.Quality = 90;
% v.FrameRate = 40;
% open(v);
% writeVideo(v,F);
% close(v);

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
fprintf('\t M3 = %f [Nm] \n',M3);
fprintf('\n');
fprintf('Link 2: \n');
fprintf('\t sigb2 = %f [MPa] \n',sigb2*1e-6);
fprintf('\t tau2 = %f [MPa] \n',tau2*1e-6);
fprintf('\t M2 = %f [Nm] \n',M2);
fprintf('\n');
fprintf('Link 1: \n');
fprintf('\t sigb1 = %f [MPa] \n',sigb1*1e-6);
fprintf('\t tau1 = %f [MPa] \n',tau1*1e-6);
fprintf('\t M1 = %f [Nm] \n',M1);