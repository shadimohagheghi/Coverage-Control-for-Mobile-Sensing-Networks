function plot_r_visible_voronoi_diagrams_V3(vor,N,x,y)

for e1=1:N
        tx=vor{e1,1};...
        ty=vor{e1,2};...
        if ~isempty(tx)
        plot(tx,ty,'g')
        end
        tx=[];...
        ty=[];...    
end
for gi=1:1:N
        plot(x(gi),y(gi),'ko','markersize',2,'markerfacecolor','k');
end   