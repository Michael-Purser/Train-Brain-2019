function path = trajInterpolate(P,n)
% Returns piecewise 3D linear path between waypoints defined in P.
% P must contain begin and endpoints.
% Between waypoints, n points used for interpolation.

N    = n*(size(P,2)-1);
path = zeros(3,N);

for i=1:size(P,2)-1
    current = P(:,i);
    next    = P(:,i+1);
    part    = [linspace(current(1),next(1),n);linspace(current(2),next(2),n);...
                    linspace(current(3),next(3),n)];
                
    path(:,(i-1)*n+1:i*n) = part;
end