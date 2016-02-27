function vor=power_diagrams_v6(x,y,x1,y1,N,r)
vor={};...
minx=min(x1);miny=min(y1);...
maxx=max(x1);maxy=max(y1);...
for j1=1:N
    vor{j1,1}=x1;...%tha allaxtei se vispol
    vor{j1,2}=y1;...
end
xtemp=x1;ytemp=y1;
for j1=1:N
    p1(1)=x(j1);p1(2)=y(j1);r1=r(j1);...

    
    for j2=j1+1:N
      if j2~=j1
%             vor{j2,1}=x1;...%tha allaxtei se vispol
%             vor{j2,2}=y1;...
        p2(1)=x(j2);p2(2)=y(j2);r2=r(j2);...
       [a,b,c]=line_power_voronoi_v2(p1,p2,r1,r2);...
%        null=check_for_null_vor(a,b,c,x1,y1);%elegxw an h diaxwristikh grammh einai ektos tou xwroy endiaferontos. 
        %An einai (null==1) ,tote to vor tou komvou me thn mikroterh aktina (anamesa stis r1 kai r2 einai keno 
        
            if b~=0
                 yl1=-a*(minx-100)-c/b;
                 yl2=-a*(maxx+100)-c/b;
                
                 %Ousiastika kataskeyazw ta dio tmhmata sta opoia xwrizei ena orthogwno
                 %h diagwnios tou.
                if -b/a>0
%                      box1=[minx-10,yl1;minx-10,miny-10;maxx+10 miny-10;maxx+10,yl2;minx-10,yl1];
%                       box2=[minx-10,maxy+10;maxx+10,maxy+10;maxx+10,yl2;minx-10,yl1;minx-10,maxy+10];
                     box1=[minx-100,yl1;maxx+100,yl2;maxx+100,yl1;minx-100,yl1]; 
                     box2=[minx-100,yl1;minx-100,yl2;maxx+100,yl2;minx-100,yl1];                  
                else
%                      box1=[minx-10,yl1;minx-10,miny-10;maxx+10 miny-10;maxx+10,yl2;minx-10,yl1];
%                      box2=[minx-10,maxy+10;maxx+10,maxy+10;maxx+10,yl2;minx-10,yl1;minx-10,maxy+10];
                     box1=[minx-100,yl2;minx-100,yl1;maxx+100,yl2;minx-100,yl2]; 
                     box2=[maxx+100,yl2;maxx+100,yl1;minx-100,yl1;maxx+100,yl2]; 
                end
                
            else

                 box1=[minx-100,miny-100;minx-100,maxy+100;-c/a,maxy+100;-c/a,miny-100;minx-100,miny-100];
                 box2=[-c/a,maxy+100;-c/a,miny-100;maxx+100,miny-100;maxx+100,maxy+100;-c/a,maxy+100];  
                
                
            end
            [choose_box,not_box]=choose_halfplane_v2(a,b,c,p1,p2,box1,box2,r1,r2);
%             [vorx1,vory1]=polybool('intersection',box1(:,1),box1(:,2),x1,y1);...
%             [vorx2,vory2]=polybool('intersection',box2(:,1),box2(:,2),x1,y1);...


            %%ta temp_vorx periexei to kommati pou tha petaksw apo to
            %%vor{J1,}! KAI EINAI TO KOMMATTI POU THA KRATHSEI O J2!
            
            [temp_vorx,temp_vory]=polybool('intersection',not_box(:,1),not_box(:,2),xtemp,ytemp); %x1,y1 tha antikatastathoun me to vispol
%             inp=inpolygon(p1(1),p1(2),vorx1,vory1);...
            tx=vor{j1,1};ty=vor{j1,2};...
            [xtemp,ytemp]=polybool('-',tx,ty,temp_vorx,temp_vory);...
            vor{j1,1}=xtemp;vor{j1,2}=ytemp;...
             vor{j2,1}=temp_vorx;vor{j2,2}=temp_vory;...
      end 
    end
end


%vor
% for t=1:N
%     tx=vor{t,1};ty=vor{t,2};
%     plot(tx,ty,'r')
%     hold on
%     k=360:1;
%     c2=r(t)*cosd(k)+x(t);
%     d2=r(t)*sind(k)+y(t);
%     plot(c2,d2,'b')
% end






return