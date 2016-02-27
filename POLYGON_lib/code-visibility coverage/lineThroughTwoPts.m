%  --------------------------- LineThroughTwoPts --------------------------
%  [a,b,c]=lineThroughTwoPts(p1,p2)  
%  Computes lines through two points. Warning when p1 = p2 *)
%  The line is expressed as a triple {a,b,c} corresponding to the equation a x + b y + c ==0 *)

function [a,b,c]=lineThroughTwoPts(p1,p2)
if norm(p1-p2)==0
    warning('No unique line through same point.')
end
x1=p1(1);y1=p1(2);x2=p2(1);y2=p2(2);
if abs(x2*y1-x1*y2)>0
    a=(y2-y1)/(x2*y1-x1*y2);
    b=(x1-x2)/(x2*y1-x1*y2);
    c=1;
else
    c=0;
    if abs(x1)>0
        a=-y1/x1;b=1;
    elseif abs(x2)>0
        a=-y2/x2;b=1;
    elseif abs(y1)>0
        b=-x1/y1;a=1;
    elseif abs(y2)>0
        b=-x2/y2;a=1;
    end
end
k=sqrt(a^2+b^2);
a=a/k;b=b/k;c=c/k;
