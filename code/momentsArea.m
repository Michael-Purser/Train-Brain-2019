function [I,H] = momentsArea(h,b,t)

I = 1/12*(b-2*t)*t^3 + (b-2*t)*t*((h-t)/2)^2 + ...
    2*(1/12*(h/2)^3*t + h/2*t*(h/4)^2);
I = 2*I;

H = (b-2*t)*t*(h-t)/2 + 2*h/2*t*(h/4);
end