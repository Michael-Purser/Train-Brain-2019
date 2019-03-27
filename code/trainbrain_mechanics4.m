clear;
close all;
clc;

% Data
wl = 7.76;      % luifelbreedte
wg = 0.5;       % verschil halfbreedtes luifel/perron
wo = 1.5;       % afstand tussen perron rand en midden van rails
hl = 3.9;       % hoogte bovenkant luifel (vanaf perron)
ho = 3.5;       % hoogte onderkant kabels

sigb = 218e6;   % maximaal toelaatbare buigspanning voor staal S235JR
tau = 126e6;    % maximaal toelaatbare schuifspanning voor staal S235JR  
E = 210e9;      % E-modulus staal
g = 9.81;       % gravity
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


% Andere parameters:
Cdesign = [0;wg+wo;hl+0.7];     % verste punt dat de arm zeker moet kunnen halen
start   = [-2;0;0];             % startpositie deployment



%% GEOMETRIE

L   = sqrt((Cdesign(2))^2+(Cdesign(3))^2);
L1  = 1.05*0.5*L;   % 5% marge zodat arm in verste punt niet sigulair is
L2  = L1;           % beide links zelfde lengte
L3  = 0.5*wl;       % link 3 moet half de luifelbreedte zijn

fprintf('Geometry: \n');
fprintf('\t L1 \t %f [m] \n',L1);
fprintf('\t L2 \t %f [m] \n',L2);
fprintf('\t L3 \t %f [m] \n',L3);
fprintf('\n')



%% STERKTE & STIJFHEID

factor = 1.5;

% Link3:

[I3,H3]     = momentsArea(h3,b3,t3);
ymax3       = h3/2;
G3          = -mL3*L3*g;
R3          = -G3;
M3          = 0.5*L3*G3;
Vmax3       = factor*abs(R3);
Mmax3       = factor*M3;
sigb3       = abs(Mmax3)*ymax3/I3;
tau3        = Vmax3*H3/(I3*t3);

f3          = 1/3*abs(G3)*L3^3/(E*I3);
phi3        = 0.5*abs(G3)*(L3)^2/(E*I3);
f3          = f3+0.5*L*sin(phi3);

% Link2:

[I2,H2]     = momentsArea(h2,b2,t2);
ymax2       = h2/2;
G2          = -mL2*L2*g;
GC          = -mC*g;
R2          = -GC + R3 - G2;
M2          = 0.5*L2*G2 + L2*(GC-R3)+M3;
Vmax2       = factor*abs(R2);
Mmax2       = factor*M2;
sigb2       = abs(Mmax2)*ymax2/I2;
tau2        = Vmax2*H2/(I2*t2);

% Link2:

[I1,H1]     = momentsArea(h1,b1,t1);
ymax1       = h1/2;
G1          = -mL1*L1*g;
GB          = -mB*g;
R1          = -GB + R2 - G1;
M1          = 0.5*L1*G1 + L1*(GB-R2)+M2;
Vmax1       = factor*abs(R1);
Mmax1       = factor*M1;
sigb1       = abs(Mmax1)*ymax1/I1;
tau1        = Vmax1*H1/(I1*t1);


fprintf('Strength and stiffness: \n');
fprintf('\t sigb \t %f [MPa] \n',sigb*1e-6);
fprintf('\t tau \t %f [MPa] \n',tau*1e-6);
fprintf('\n');
fprintf('   Link 3: \n');
fprintf('\t sigb3 \t %f [MPa] \n',sigb3*1e-6);
fprintf('\t tau3 \t %f [MPa] \n',tau3*1e-6);
fprintf('\t f3 \t %f [m] \n',f3);
fprintf('\t M3 \t %f [Nm] \n',M3);
fprintf('\n');
fprintf('   Link 2: \n');
fprintf('\t sigb2 \t %f [MPa] \n',sigb2*1e-6);
fprintf('\t tau2 \t %f [MPa] \n',tau2*1e-6);
fprintf('\t M2 \t %f [Nm] \n',M2);
fprintf('\n');
fprintf('   Link 1: \n');
fprintf('\t sigb1 \t %f [MPa] \n',sigb1*1e-6);
fprintf('\t tau1 \t %f [MPa] \n',tau1*1e-6);
fprintf('\t M1 \t %f [Nm] \n',M1);
fprintf('\n');



%% TRAJECT
% Referentie posities zijn gegeven voor de positie van punt C, dat niet het
% eindpunt is van de eind-effector, maar de joint tussen link 2 en 3 !!!
% Cleaning begint op x=0; alle deployement enz. moet gebeuren met punt A op
% x<0.

load('xzycoor.mat');
xzycoor         = xzycoor(1:1400,:)';   % selecteer relevante deel
xyzcoor         = [xzycoor(1,:);xzycoor(3,:);xzycoor(2,:)];
xyzcoor(2,:)    = xyzcoor(2,:) - L3;

ref_cleaning    = xyzcoor;
n_cleaning      = size(ref_cleaning,2);
T_cleaning      = 10;
vx_cleaning     = (ref_cleaning(1,n_cleaning)-ref_cleaning(1,1))/T_cleaning;
A_cleaning      = [linspace(ref_cleaning(1,1),ref_cleaning(1,n_cleaning),n_cleaning);...
                    zeros(1,n_cleaning);zeros(1,n_cleaning)];

P_deploy        = [[0.1;0;0.5]+start ref_cleaning(:,1)+start];
ref_deploy      = trajInterpolate(P_deploy,100);
n_deploy        = size(ref_deploy,2);
A_deploy        = start;

P_move          = [ref_deploy(:,end) ref_cleaning(:,1)];
ref_move        = trajInterpolate(P_move,100);
n_move          = size(ref_move,2);
T_move          = 2;
vx_move         = (ref_move(1,n_move)-ref_move(1,1))/T_move;
A_move          = [linspace(ref_move(1,1),ref_move(1,n_move),n_move);...
                    zeros(1,n_move);zeros(1,n_move)];

ref_total       = [ref_deploy ref_move ref_cleaning];
n_total         = size(ref_total,2);

tol             = 1e-5;

Arm             = zeros(3,4,n_total); % initialiseer Arm
X               = zeros(n_total,3);   % initialiseer X


% vind posities voor verschillende stappen

[X,Arm] = trajSolve(ref_total,'initial deployment','deploy',1,n_deploy,A_deploy,tol,L1,L2,L3,X,Arm);
[X,Arm] = trajSolve(ref_total,'move 1','move',n_deploy+1,n_deploy+n_move,...
                A_move,tol,L1,L2,L3,X,Arm);
[X,Arm] = trajSolve(ref_total,'cleaning 1','cleaning',n_deploy+n_move+1,...
                n_deploy+n_move+n_cleaning,A_cleaning,tol,L1,L2,L3,X,Arm);

            
% plot 1 positie:
k = 700;
plotPath(k,Arm,X,h1,b1,L1,h2,b2,L2,h3,b3,L3,wg,wo,wl);


% maak filmpje; per 100 frames --> laat toe grotere resolutie ste halen
% acteraf filmpje samenstellen met extern programma (e.g. 'Shotcut' op
% linux).

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


