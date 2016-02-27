function [a,b,c]=line_power_voronoi_v2(p1,p2,r1,r2)
%function [a,b,c]=lineSegmentBisector(p1,p2)


% 

k=1:360;
c1=r1*cosd(k)+p1(1);
d1=r1*sind(k)+p1(2);
c2=r2*cosd(k)+p2(1);
d2=r2*sind(k)+p2(2);
[xm1,ym1]=polyxpoly(c1,d1,c2,d2);



if p1(2)~=p2(2) %%==>L~=inf
    b=1;...
    a=(p2(1)-p1(1))/(p2(2)-p1(2));...%klish
    %c=-(  p1(2)+p2(2)+a*(p1(1)+p2(1))   )/2;...
    if ~isempty(xm1) %an uparxei tomh metaksu twn  duo kuklwn
        xm=xm1(1);ym=ym1(1);...
    else %an den uparxei tomh metaksu twn duo kuklwn
        de=sqrt( (p1(1)-p2(1))^2  +   (p1(2)-p2(2))^2    );...
        if de<max(r1,r2)%o mikros kuklos oloklhros mesa ston megalo
            [rm,ind]=min([r1,r2]);
             k1=(de/2) + (r1^2  -  r2^2)/(2*de);...
            if ind==1
                z=(p1(1)-p2(1))+(p1(2)-p2(2))*1i;...
            else
                %ind==2
                z=(p2(1)-p1(1))+(p2(2)-p1(2))*1i;...
            end
           
            th=angle(z);
            xm=p1(1)+abs(k1)*cos(th);
            ym=p1(2)+abs(k1)*sin(th);
        else% an o mikros einai oloklhros eksw apo ton megalo kuklo
            k1=(de/2) + (r1^2  -  r2^2)/(2*de);...
            z=(p2(1)-p1(1))+(p2(2)-p1(2))*1i;...
            th=angle(z);
            xm=p1(1)+abs(k1)*cos(th);
            ym=p1(2)+abs(k1)*sin(th);            
        end
    end

    c=-a*xm-b*ym;...  
else %idia y

    a=1;b=0;%c=-(p1(1)+p2(1))/2;
    de=sqrt( (p1(1)-p2(1))^2  +   (p1(2)-p2(2))^2    );...
    k1=(de/2) + (r1^2  -  r2^2)/(2*de);...
    z=(p1(1)-p2(1))+(p1(2)-p2(2))*1i;...
    th=angle(z);

    
    if ~isempty(xm1) %an uparxei tomh metaksu twn  duo kuklwn
        xm=xm1(1);ym=ym1(1);...
    else %an den uparxei tomh metaksu twn duo kuklwn
        de=sqrt( (p1(1)-p2(1))^2  +   (p1(2)-p2(2))^2    );...
        if de<max(r1,r2)%o mikros kuklos oloklhros mesa ston megalo
            [rm,ind]=min([r1,r2]);
             k1=(de/2) + (r1^2  -  r2^2)/(2*de);...
            if ind==1
                z=(p1(1)-p2(1))+(p1(2)-p2(2))*1i;...
            else
                %ind==2
                z=(p2(1)-p1(1))+(p2(2)-p1(2))*1i;...
            end
           
            th=angle(z);
            xm=p1(1)+abs(k1)*cos(th);
            ym=p1(2)+abs(k1)*sin(th);
        else% an o mikros einai oloklhros eksw apo ton megalo kuklo
            k1=(de/2) + (r1^2  -  r2^2)/(2*de);...
            z=(p2(1)-p1(1))+(p2(2)-p1(2))*1i;...
            th=angle(z);
            xm=p1(1)+abs(k1)*cos(th);
            ym=p1(2)+abs(k1)*sin(th);            
        end
    end
    %%%%%%
    %%%%%%
    c=-xm;

end