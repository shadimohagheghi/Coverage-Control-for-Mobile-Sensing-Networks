function point=halfLineWithLineSegmentIntersection(paopen,paclosed,pb1,pb2,abc_a,abc_b)
%function point=halfLineWithLineSegmentIntersection(paopen,paclosed,pb1,pb2,abc_a,abc_b)

%halfLine with Segment intersection, where the coefficients of the two lines are
%given. The half line is defined by the points paopen,paclosed and
%itscoefficients are given in abc_a. The half line extends from point
%paclosed to paopen. The line segment is defined by the points pb1,pb2 and
%the line coefficients are iven in abc_b.
%If the coefficients abc_a or abc_b are not given, they are computed.
if isempty(abc_a)
    [a,b,c]=lineThroughTwoPts(paopen,paclosed);
    abc_a=[a,b,c];
end
if isempty(abc_b)
    [a,b,c]=lineThroughTwoPts(pb1,pb2);
    abc_b=[a,b,c];
end

Tolerance=0.001;
a1=abc_a(1);b1=abc_a(2);c1=abc_a(3);
a2=abc_b(1);b2=abc_b(2);c2=abc_b(3);

xb=[pb1(1),pb2(1)];
yb=[pb1(2),pb2(2)];

A=[a1 b1;a2 b2];
if det(A)==0
    point=[];
    return
else
    C=[-c1;-c2];
    point=A\C;
    if point(1)-min(xb)<-Tolerance||point(1)-max(xb)>Tolerance||point(2)-min(yb)<-Tolerance||point(2)-max(yb)>Tolerance
        point=[];
    else
        if paopen(1)>paclosed(1)
            if point(1)<paclosed(1)
                point=[];
                return
            end
        else
            if point(1)>paclosed(1)
                point=[];
                return
            end
        end
        if paopen(2)>paclosed(2)
            if point(2)<paclosed(2)
                point=[];
                return
            end
        else
            if point(2)>paclosed(2)
                point=[];
                return
            end
        end    
    end
end