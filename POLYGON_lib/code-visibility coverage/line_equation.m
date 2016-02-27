function [L,B]=line_equation(x,y)
%x,y: coordinates of two points 
%y=Lx+B 
if abs(x(1)-x(2))>=0
    L=(y(1)-y(2))/(x(1)-x(2));...
    B=y(1)-L*x(1);...    
else
L=Inf;...
    
end



return