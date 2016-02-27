function plot_r_visible_voronoi_diagrams_V2(r_lim,N,x,y)

for e1=1:N
        tx=r_lim{e1,1};...
        ty=r_lim{e1,2};... 
        if ~isempty(tx)
        plot(tx,ty,'red')
        end
        tx=[];...
        ty=[];...    
end
for gi=1:1:N
        plot(x(gi),y(gi),'ko','markersize',2,'markerfacecolor','k');
end   