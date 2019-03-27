function [X,Arm] = trajSolve(stageRef,stageName,stageType,stageN,stageBasePos,tol,L1,L2,L3)

Arm             = zeros(3,4,stageN); % initialiseer Arm
X               = zeros(stageN,3);   % initialiseer X

for k = 1:stageN

    % iteratie log:
    fprintf('Iteration %i of %s \n',k,stageName);

    % positie van punt C:
    ref = stageRef(:,k);

    % positie van punt A:
    % in deploy/reploy stap blijft wagon stil --> A beweegt niet
    % in andere stappen beweegt A wel
    if strcmp(stageType,'deploy')==1
        A   = stageBasePos;
    elseif strcmp(stageType,'move')==1
        A   = stageBasePos(:,k);
    elseif strcmp(stageType,'cleaning')==1
        A   = stageBasePos(:,k);
    else
        error('unrecognized stage type');
    end

    % bepaal hoek psi; hangt af van type beweging:
    l = sqrt((ref(2)-A(2))^2 + (ref(1)-A(1))^2);
    if l < tol
        if strcmp(stageType,'deploy')==1
            psi = pi/2;
        elseif strcmp(stageType,'move')==1
            psi = 0;
        elseif strcmp(stageType,'cleaning')==1
            psi = 0;
        else
            error('unrecognized stage type');
        end
    else
        psi = acos((ref(2)-A(2))/l);
        % tekencorrectie door cosinus:
        if ref(1)<A(1)
            psi = -psi;
        end
    end
    
    % begingok hoeken; hangt af van type beweging en van iteratienummer:
    if k==1
        if strcmp(stageType,'deploy')==1
            x0 = [0,0];
        elseif strcmp(stageType,'move')==1
            x0 = [0,pi/2];
        elseif strcmp(stageType,'cleaning')==1
            x0 = [0,pi/2];
        else
            error('unrecognized stage type');
        end
    else
        x0 = 0.9*X(k-1,1:2);
    end
    
    % oplossen naar hoeken:
    options = optimset('Display','off');
    [x,~,exitflag] = fsolve(@(x) armGeom(x,A,ref,psi,L1,L2),x0,options);
    if (exitflag ~= 1)
        error('The fsolve exit flag was not 1, probably no convergence!');
    end
    
    % vul oplossing aan:
    X(k,:) = [x psi];

    B = A + [(L1*sin(x(1)))*sin(psi);(L1*sin(x(1)))*cos(psi);L1*cos(x(1))];
    C = B + [(L2*sin(x(2)))*sin(psi);(L2*sin(x(2)))*cos(psi);L2*cos(x(2))];
    % positie van punt D hangt af van stage: gedurende deploy/reploy volgt
    % het de hoek psi, anders niet:
    if strcmp(stageType,'deploy')==1
        D = C + [L3*sin(psi);L3*cos(psi);0];
    elseif strcmp(stageType,'move')==1
        D = C + [0;L3;0];
    elseif strcmp(stageType,'cleaning')==1
        D = C + [0;L3;0];
    else
        error('unrecognized stage type');
    end
    
    
    Arm(:,:,k) = [A B C D];
end

