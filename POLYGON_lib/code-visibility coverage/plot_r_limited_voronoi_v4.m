function [r_lim,r_vor]=plot_r_limited_voronoi_v4(x,y,x1,y1,N,r)
[v,c] = voronoin([x(:) y(:)]);
for e=1:1:N
    % krataw tis syntetagmenes tou voronoi tou robot #e
    vorx{e}=v(c{e},1);
    vory{e}=v(c{e},2);
    
    [x1,y1]=poly2cw(x1,y1);
    [vorx{e},vory{e}]=poly2cw(vorx{e},vory{e});
    [xv,yv]=polybool('&',x1,y1,vorx{e},vory{e});%vriskw thn tomh toy voronoi me ton xwro pou thelw na kalypsw
    %plot(xv,yv,'red','linewidth',1.3)
 
    k=360:-1:1;
    c2=r(e)*cosd(k)+x(e);
    d2=r(e)*sind(k)+y(e);
    %plot(c2,d2,'r');
    % plot(r(e)*cosd(k)+x(e),r(e)*sind(k)+y(e),'r');

    
    [xc,yc]=polybool('&',xv,yv,c2,d2);
    plot(xc,yc,'red','linewidth',1.3)

    xc={xc};...
    yc={yc};...
    r_lim{e,1}=xc;...               %tx=cell2mat(r_lim{j,1})
    r_lim{e,2}=yc;...               %j--->xc(exterior_nodes(j))
    

    xv={xv};...
    yv={yv};...
    r_vor{e,1}=xv;...               %tx=cell2mat(r_lim{j,1})
    r_vor{e,2}=yv;...               %j--->xc(exterior_nodes(j))
end
figure(1)
plot(x1,y1,'k','linewidth',1.4)
for gi=1:1:N
    plot(x(gi),y(gi),'ko','markersize',2,'markerfacecolor','k');
end
return
% for h1=1:1:N;
%
%     for h2=1:1:N;
%         distg=distance([x(h1),y(h1)],[x(h2),y(h2)]);
%         %dist3(h1,h2)=distg;
%         if (distg<=R);
%           %  pause
%             plot([x(h1),x(h2)],[y(h1),y(h2)],'g');
%           % pause;
%         end
%
%     end
% end