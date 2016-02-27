function vcell = voronoi_limited_2centers(p,pol)
%Ypologizei to voronoi dio kentrwn mesa se ena polygwno.
%An to polygwno einai koilo, paragei pollaples (disjoint) perioxes.
if size(p,1)~=2
    error('Prepei na dothoun dio shmeia eisodou mono')
end

minx=min(pol(:,1));
maxx=max(pol(:,1));
miny=min(pol(:,2));
maxy=max(pol(:,2));

%Ypologizw thn mesokatheto twn dio shmeiwn.
[a,b,c]=lineSegmentBisector(p(1,:),p(2,:));

%Kataskewazw dio 'koutia' pou enwnontai me thn mesokatheto kai periexoun
%sinolika olo to polygwno. To kathe ena apo auta ta koutia periexei ena kai
%mono kentro.
if b~=0
    y1=-a*(minx-10)-c/b;
    y2=-a*(maxx+10)-c/b;

    %Ousiastika kataskeyazw ta dio tmhmata sta opoia xwrizei ena orthogwno
    %h diagwnios tou.
    if -b/a>0
        box1=[minx-10,y1;minx-10,miny-10;maxx+10 miny-10;maxx+10,y2];
        box2=[minx-10,maxy+10;maxx+10,maxy+10;maxx+10,y2;minx-10,y1];
    else
        box1=[minx-10,y1;minx-10,miny-10;maxx+10 miny-10;maxx+10,y2];
        box2=[minx-10,maxy+10;maxx+10,maxy+10;maxx+10,y2;minx-10,y1];
    end
else
    box1=[minx-10,miny-10;minx-10,maxy+10;-c/a,maxy+10;-c/a,miny-10];
    box2=[-c/a,maxy+10;-c/a,miny-10;maxx+10,miny-10;maxx+10,maxy+10];
end
% 
% plot(polx,poly,'g')
% hold on
% plot([box1(:,1);box1(1,1)],[box1(:,2);box1(1,2)],'b')
% plot(box2(:,1),box2(:,2),'r')
% plot(p(:,1),p(:,2),'m+')
[vorx1,vory1]=polybool('intersection',box1(:,1),box1(:,2),pol(:,1),pol(:,2));
[vorx2,vory2]=polybool('intersection',box2(:,1),box2(:,2),pol(:,1),pol(:,2));
inp=inpolygon(p(:,1),p(:,2),vorx1,vory1);
if inp(1)==1
    vcell(1)={[vorx1,vory1]};
    vcell(2)={[vorx2,vory2]};
else
    vcell(1)={[vorx2,vory2]};
    vcell(2)={[vorx1,vory1]};
end
    
end
% 
% vc1=vcell{1}
% plot(polx,poly,'g')
% hold on
% plot(p(1,1),p(1,2),'b+')
% plot(vc1(:,1),vc1(:,2),'b')
% 
% 
% vc2=vcell{2}
% plot(polx,poly,'g')
% hold on
% plot(p(2,1),p(2,2),'r+')
% plot(vc2(:,1),vc2(:,2),'r')
