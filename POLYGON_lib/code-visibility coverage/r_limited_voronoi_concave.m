function [r_lim,r_vor]=r_limited_voronoi_concave(x,y,x1,y1,N,r)
[v,c] = voronoin([x(:) y(:)]);
for e=1:1:N
    % krataw tis syntetagmenes tou voronoi tou robot #e
    vorx{e}=v(c{e},1);
    vory{e}=v(c{e},2);
    
    [x1,y1]=poly2cw(x1,y1);
    [vorx{e},vory{e}]=poly2cw(vorx{e},vory{e});
    [xv,yv]=polybool('&',x1,y1,vorx{e},vory{e});%vriskw thn tomh toy voronoi me ton xwro pou thelw na kalypsw
    [x_pol,y_pol]=visible_polygon(x,y,x1,y1,e,r);...
    [xv_vis,yv_vis]=polybool('&',x_pol,y_pol,yv,xv);    
    k=360:-1:0;
    c2=r(e)*cosd(k)+x(e);...
    d2=r(e)*sind(k)+y(e);...
    %plot(c2,d2,'r');
    % plot(r(e)*cosd(k)+x(e),r(e)*sind(k)+y(e),'r');

    
    [xc,yc]=polybool('&',xv_vis,yv_vis,c2,d2);
     plot(xc,yc)
 
    xc={xc};...
    yc={yc};...
    r_lim{e,1}=xc;...               %tx=cell2mat(r_lim{j,1})
    r_lim{e,2}=yc;...               %j--->xc(exterior_nodes(j))
    

    xv_vis={xv_vis};...
    yv_vis={yv_vis};...
    r_vor{e,1}=xv_vis;...               %tx=cell2mat(r_lim{j,1})
    r_vor{e,2}=yv_vis;...               %j--->xc(exterior_nodes(j))
end

return
