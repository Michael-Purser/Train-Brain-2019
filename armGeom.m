function F = armGeom(x,A,Cgoal,psi,L1,L2)

F(1) = L1*cos(x(1))+L2*cos(x(2))-(Cgoal(3)-A(3));
F(2) = (L1*sin(x(1))+L2*sin(x(2)))*cos(psi)-(Cgoal(2)-A(2));
F(3) = (L1*sin(x(1))+L2*sin(x(2)))*sin(psi)-(Cgoal(1)-A(1));

end