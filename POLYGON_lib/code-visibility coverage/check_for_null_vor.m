function null=check_for_null_vor(a,b,c,x1,y1) %ta x1,y1 einai panta olos o xwros endiaferontos

null=0;%den tha einai keno to vor
if b~=0
    xcheck=-100:100;
    ycheck=-(-a*x+c)/b;
    [xn,yn]=polyxpoly(xcheck,ycheck,x1,y1);
    if isempty(xn) %h diaxwristikh grammh einai ektos tou xwrou
        null=1;%to vor tha einai keno
    end
else
    ycheck=-100:100;
    xcheck=-c;
    [xn,yn]=polyxpoly(xcheck,ycheck,x1,y1);
    if isempty(xn) %h diaxwristikh grammh einai ektos tou xwrou
        null=1;%to vor tha einai keno
    end
end
return