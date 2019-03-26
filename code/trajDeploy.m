function Cgoals = trajDeploy(P,n)

Cgoals = P(:,1);

for i=1:size(P,2)-1
    start = Cgoals(:,end);
    next = P(:,i+1);
    cgoals = [linspace(start(1),next(1),n);linspace(start(2),next(2),n);...
        linspace(start(3),next(3),n)];
    Cgoals = [Cgoals cgoals];
end