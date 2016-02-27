function vcell = power_voronoi_limited_2centers_v2(p_,pol,r,p)
%Ypologizei to voronoi dio kentrwn mesa se ena polygwno.
%An to polygwno einai koilo, paragei pollaples (disjoint) perioxes.
if size(p_,1)~=2
    error('Prepei na dothoun dio shmeia eisodou mono')
end

minx=min(pol(:,1));
maxx=max(pol(:,1));
miny=min(pol(:,2));
maxy=max(pol(:,2));

%Ypologizw thn mesokatheto twn dio shmeiwn.
% [a,b,c]=lineSegmentBisector(p(1,:),p(2,:));
j11=find(p_(1,1)==p(:,1));... %o j1 komvos antistoixei ston komvo 1 tou sunolou p_
j12=find(p_(1,2)==p(:,2));...
j1=intersect(j11,j12);...
p1(1)=p(j1,1);p1(2)=p(j1,2);
r1=r(j1);



j21=find(p_(2,1)==p(:,1));
j22=find(p_(2,2)==p(:,2));
j2=intersect(j21,j22);...
p2(1)=p(j2,1);p2(2)=p(j2,2);
r2=r(j2);...
    
[a,b,c]=line_power_voronoi_v2(p1,p2,r1,r2);


%Kataskewazw dio 'koutia' pou enwnontai me thn mesokatheto kai periexoun
%sinolika olo to polygwno. To kathe ena apo auta ta koutia periexei ena kai
%mono kentro.
if b~=0
                 yl1=-a*(minx-100)-c/b;...
                 yl2=-a*(maxx+100)-c/b;...
                
                 % Ftiaxnw ta duo kommatia pou dhmiourgei h diagwnios enos
                 % orthogoniou
                if -b/a>0 &&  abs(-b/a)~=Inf
%                      box1=[minx-10,yl1;minx-10,miny-10;maxx+10 miny-10;maxx+10,yl2;minx-10,yl1];
%                      box2=[minx-10,maxy+10;maxx+10,maxy+10;maxx+10,yl2;minx-10,yl1;minx-10,maxy+10];
%                      box1=[minx-100,yl1;maxx+100,yl2;maxx+100,yl1;minx-100,yl1];...
%                      box2=[minx-100,yl1;minx-100,yl2;maxx+100,yl2;minx-100,yl1];...   
                       box1=[minx-100,yl1;minx-100,miny-100;maxx+100,miny-100;maxx+100,yl2;minx-100,yl1];...%katw kouti
                       box2=[minx-100,yl1;minx-100,maxy+100;maxx+100,maxy+100;maxx+100,yl2;minx-100,yl1];...   %anw kouti
                elseif -b/a<0 &&  abs(-b/a)~=Inf
%                      box1=[minx-10,yl1;minx-10,miny-10;maxx+10 miny-10;maxx+10,yl2;minx-10,yl1];
%                      box2=[minx-10,maxy+10;maxx+10,maxy+10;maxx+10,yl2;minx-10,yl1;minx-10,maxy+10];
%                      box1=[minx-100,yl2;minx-100,yl1;maxx+100,yl2;minx-100,yl2];... 
%                      box2=[maxx+100,yl2;maxx+100,yl1;minx-100,yl1;maxx+100,yl2];... 
                       box1=[minx-100,miny-100;maxx+100,miny-100;maxx+100,yl2;minx-100,yl1;minx-100,miny-100];... %katw kouti
                       box2=[maxx+100,yl2;maxx+100,maxy+100;minx-100,maxy+100;minx-100,yl1;maxx+100,yl2];...%anw kouti
                elseif a==0
                     box1=[minx-100,miny-100;minx-100,-c/b;maxx+100,-c/b;maxx+100,miny-100;minx-100,miny-100];... 
                     box2=[minx-100,-c/b;maxx+100,-c/b;maxx+100,maxy+100;minx-100,maxy+100;minx-100,-c/b];... 
                end
                
            else

                 box1=[minx-100,miny-100;minx-100,maxy+100;-c/a,maxy+100;-c/a,miny-100;minx-100,miny-100];...
                 box2=[-c/a,maxy+100;-c/a,miny-100;maxx+100,miny-100;maxx+100,maxy+100;-c/a,maxy+100];...  
                
                
 end
% 
% plot(polx,poly,'g')
% hold on
% plot([box1(:,1);box1(1,1)],[box1(:,2);box1(1,2)],'b')
% plot(box2(:,1),box2(:,2),'r')
% plot(p(:,1),p(:,2),'m+')

[in_box,out_box]=choose_halfplane_v2(a,b,c,p1,p2,box1,box2,r1,r2);...%in_box: einai to "kouti/voronoi_cell" pou tha anetethei sto j1 (sugkrinontas mono tous komvous j1 kai j2)
    


[vorx1,vory1]=polybool('intersection',in_box(:,1),in_box(:,2),pol(:,1),pol(:,2));%j1
[vorx2,vory2]=polybool('intersection',out_box(:,1),out_box(:,2),pol(:,1),pol(:,2));%j2

% 
% inp=inpolygon(p(:,1),p(:,2),vorx1,vory1);
% if inp(1)==1
    vcell(1)={[vorx1,vory1]};%o j1 komvos antistoixei ston komvo 1 tou sunolou p_
    vcell(2)={[vorx2,vory2]};
% else
%     vcell(1)={[vorx2,vory2]};
%     vcell(2)={[vorx1,vory1]};
% end
    
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
