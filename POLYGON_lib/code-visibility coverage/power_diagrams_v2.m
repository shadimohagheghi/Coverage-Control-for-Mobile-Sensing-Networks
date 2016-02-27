function vor=power_diagrams_v2(x,y,x1,y1,N,r)
vor={};...
minx=min(x1);miny=min(y1);...
maxx=max(x1);maxy=max(y1);...



for j1=1:N
    p1(1)=x(j1);p1(2)=y(j1);r1=r(j1);...
    vor{j1,1}=x1;...%tha allaxtei se vispol
    vor{j1,2}=y1;...
    for j2=1:N
      if j2~=j1
%             vor{j2,1}=x1;...%tha allaxtei se vispol
%             vor{j2,2}=y1;...
        	p2(1)=x(j2);p2(2)=y(j2);r2=r(j2);...
            [a,b,c]=line_power_voronoi(p1,p2,r1,r2);...
            if b~=0
                 yl1=-a*(minx-10)-c/b;
                 yl2=-a*(maxx+10)-c/b;
                 %Ousiastika kataskeyazw ta dio tmhmata sta opoia xwrizei ena orthogwno
                 %h diagwnios tou.
                if -b/a>0
                     box1=[minx-10,yl1;minx-10,miny-10;maxx+10 miny-10;maxx+10,yl2;minx-10,yl1];
                     box2=[minx-10,maxy+10;maxx+10,maxy+10;maxx+10,yl2;minx-10,yl1;minx-10,maxy+10];
                else
                     box1=[minx-10,yl1;minx-10,miny-10;maxx+10 miny-10;maxx+10,yl2;minx-10,yl1];
                     box2=[minx-10,maxy+10;maxx+10,maxy+10;maxx+10,yl2;minx-10,yl1;minx-10,maxy+10];
                end
                
            else

                 box1=[minx-10,miny-10;minx-10,maxy+10;-c/a,maxy+10;-c/a,miny-10;minx-10,miny-10];
                 box2=[-c/a,maxy+10;-c/a,miny-10;maxx+10,miny-10;maxx+10,maxy+10;-c/a,maxy+10];  
                
                
            end
            [vorx1,vory1]=polybool('intersection',box1(:,1),box1(:,2),x1,y1);...
            [vorx2,vory2]=polybool('intersection',box2(:,1),box2(:,2),x1,y1);...
            
            inp=inpolygon(p1(1),p1(2),vorx1,vory1);...
                %dialexe shmeio ,elekse an anhkei sto vorx1 h sto vorx2.
                %Estw oti anhkei sto vorx1... Elekse an ikanopoieitai h
                %eksiswsh twn power diagrams gia kathe allo komvo.. an nai
                %tote dwse san power vor cell to vorx1...alliws to vorx2
            if inp==1
               temp_vor{j1,1}=vorx1;temp_vor{j1,2}=vory1;...
               temp_vor{j2,1}=vorx2;temp_vor{j2,2}=vory2;...
               %---j1----------------
               tx=vor{j1,1};ty=vor{j1,2};...
               [xtemp,ytemp]=polybool('-',tx,ty,vorx2,vory2);...
               vor{j1,1}=xtemp;vor{j1,2}=ytemp;...
               %----------------------
               
%                %---j2-----------------
%                tx=vor{j2,1};ty=vor{j2,2};...
%                [xtemp,ytemp]=polybool('-',tx,ty,vorx1,vory1);...
%                vor{j2,1}=xtemp;vor{j2,2}=ytemp;...               
%                %----------------------
           
            else
               temp_vor{j1,1}=vorx2;temp_vor{j1,2}=vory2;
               temp_vor{j2,1}=vorx1;temp_vor{j2,2}=vory1;
               %---j1----------------
               tx=vor{j1,1};ty=vor{j1,2};...
               [xtemp,ytemp]=polybool('-',tx',ty',vorx1,vory1);...
               vor{j1,1}=xtemp;vor{j1,2}=ytemp;...
               %----------------------
           
%                %---j2-----------------
%                tx=vor{j1,1};ty=vor{j1,2};...
%                [xtemp,ytemp]=polybool('-',tx,ty,vorx2,vory2);...
%                vor{j2,1}=xtemp;vor{j2,2}=ytemp;...               
%                %----------------------
            end
      end 
    end
end


%vor
for t=1:N
    tx=vor{t,1};ty=vor{t,2};
    plot(tx,ty,'r')
    hold on
    k=360:1;
    c2=r(t)*cosd(k)+x(t);
    d2=r(t)*sind(k)+y(t);
    plot(c2,d2,'b')
end






return