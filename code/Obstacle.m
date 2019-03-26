clear;close all;
%making xz obstacle
x1=linspace(0,2.99,300);
x2=linspace(3,5.99,300);
z1=linspace(3.9,2.91,100);
x3=linspace(6,8.99,300);
z2=linspace(2.9,3.89,100);
x4=linspace(9,9.99,100);
z3=linspace(3.9,4.39,50);
x5=linspace(10,10.99,100);
z4=linspace(4.4,3.91,50);
x6=linspace(11,13.99,300);
z5=linspace(3.9,3.71,20);

f1=@(x) 3.9+0.3*x;
f2=@(x) 4.8-0.3*(x-3);
f3=@(x) 6+0*x;
f4=@(x) 2.9+0*x;
f5=@(x) 9+0*x;
f6=@(x) 3.9+0*x;
f7=@(x) 10+0*x;
f8=@(x) 4.4+0*x;
f9=@(x) 11+0*x;
f10=@(x) 3.9+0*x;
f11=@(x) 14+0*x;
f12=@(x) (5-sqrt(0.3*0.3-(x-3).^2)).*(x>=2.7)+5.*(x<2.7);
f13=@(x)(5-sqrt(0.3*0.3-(x-3).^2)).*(x<=3.3)+5.*(x>3.3);

x=[x1 x2 f3(z1) x3 f5(z2) x4 f7(z3) x5 f9(z4) x6 f11(z5)];
y=[f12(x1) f13([x2 f3(z1) x3 f5(z2) x4 f7(z3) x5 f9(z4) x6 f11(z5)])];
z=[f1(x1) f2(x2) z1 f4(x3) z2 f6(x4) z3 f8(x5) z4 f10(x6) z5]; 
figure;plot(x,z);
axis equal;
hold on
time=0;
save('Time','time');
save('xz','x','z');
%making time vector and computing the result of the control
n=length(x);
xzycoor=zeros(n,3);
pos=[0;4.5;5];
t=0;
save('T','t');
for i=1:n-1
    xzycoor(i,1)=pos(1,1);
    xzycoor(i,2)=pos(2,1);
    [pos(1,1),pos(2,1)]=ZMove(pos,x,z);
end
xzycoor(end,1)=pos(1,1);
xzycoor(end,2)=pos(2,1);
plot(xzycoor(:,1),xzycoor(:,2));
yt=5*ones(1,length(xzycoor));
f14=@(x) (5-sqrt(0.5*0.5-(x-3).^2)).*(x<=3.5).*(x>=2.5)+5.*((x<2.5)+(x>3.5));
jt=f14(x);
xzycoor(:,3)=jt';
save('xzycoor','xzycoor');
figure;
plot(x,y);
axis equal
hold on
plot(xzycoor(:,1),jt);
tijd=linspace(0,length(xzycoor)-1,length(xzycoor));