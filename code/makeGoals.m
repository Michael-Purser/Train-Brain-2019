function Cgoals = makeGoals(Cstart,Cend,N,sel)

switch sel
    
    % Simple following
    case 1
        Cgoals = [linspace(Cstart(1),Cend(1),N);linspace(Cstart(2),Cend(2),N);...
    linspace(Cstart(3),Cend(3),N)];

    % Following sine
    case 2
        Cgoals = [linspace(Cstart(1),Cend(1),N);linspace(Cstart(2),Cend(2),N);...
    Cstart(3)+0.2*sin(10*linspace(Cstart(1),Cend(1),N))];
        
    % Static sine
    case 3
        Cgoals = [Cstart(1)*ones(1,N);linspace(Cstart(2),Cend(2),N);...
    Cstart(3)+0.2*sin(10*linspace(Cstart(1),Cend(1),N))];
        
    % Pure static
    case 4
        Cgoals = [Cstart(1)*ones(1,N);linspace(Cstart(2),Cend(2),N);...
    linspace(Cstart(3),Cend(3),N)];

    % Square pulse
    case 5
        C1 = [linspace(Cstart(1),1/3*Cend(1),floor(N/5));Cstart(2)*ones(1,floor(N/5));...
            Cstart(3)*ones(1,floor(N/5))];
        C2 = [1/3*Cend(1)*ones(1,floor(N/5));C1(2,end)*ones(1,floor(N/5));...
            linspace(C1(3,end),C1(3,end)+0.5,floor(N/5))];
        C3 = [linspace(1/3*Cend(1),2/3*Cend(1),floor(N/5));Cstart(2)*ones(1,floor(N/5));...
            C2(3,end)*ones(1,floor(N/5))];
        C4 = [2/3*Cend(1)*ones(1,floor(N/5));Cstart(2)*ones(1,floor(N/5));...
            linspace(C3(3,end),C3(3,end)-0.5,floor(N/5))];
        C5 = [linspace(2/3*Cend(1),Cend(1),N-4*floor(N/5));Cstart(2)*ones(1,N-4*floor(N/5));...
            Cstart(3)*ones(1,N-4*floor(N/5))];
        
        
        
        Cgoals = [C1 C2 C3 C4 C5];

        
end
end
