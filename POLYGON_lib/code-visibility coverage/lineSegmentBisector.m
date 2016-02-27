function [a,b,c]=lineSegmentBisector(p1,p2)
%function [a,b,c]=lineSegmentBisector(p1,p2)

if p1(2)~=p2(2)
    b=1;
    a=(p2(1)-p1(1))/(p2(2)-p1(2));
    c=-(p1(2)+p2(2)+a*(p1(1)+p2(1)))/2;
    
else
    a=1;b=0;c=-(p1(1)+p2(1))/2;
end