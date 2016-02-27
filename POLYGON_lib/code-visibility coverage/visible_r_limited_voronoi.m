function [r_lim2,vor2]=visible_r_limited_voronoi(x,y,x1,y1,r,N,neigh_NxN,hops)


[r_lim1,vor1]=visible_vor_step1(N,hops,neigh_NxN,x1,y1,r,x,y);...
 %---------------------------------------
%find r-del based on the above partitioning      
%[subgraph,t]=subgraph_per_node_v2(e,N,hops,neigh_NxN);... %j-subgraph in n hops 
[r_lim2,vor2]=visible_vor_step2(N,hops,neigh_NxN,x1,y1,r,x,y,r_lim1);...

%----------------------------------------
plot(x1,y1,'k','linewidth',1.4)
for e1=1:N
    tx=cell2mat(r_lim2{e1,1});...
    ty=cell2mat(r_lim2{e1,2});... 
    plot(tx,ty,'red')
    tx=[];...
    ty=[];...    
end
for gi=1:1:N
    plot(x(gi),y(gi),'ko','markersize',2,'markerfacecolor','k');
end

return
