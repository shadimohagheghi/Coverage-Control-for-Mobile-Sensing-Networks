function [vor,r_lim]=visibility_power_diagrams_v3(x,y,x1,y1,N,r)
vor={};...
minx=min(x1);miny=min(y1);...
maxx=max(x1);maxy=max(y1);...

for j3=1:N
   [x_vp1,y_vp1]=visible_polygon_v2(x,y,x1,y1,j3);%visibility_polygon-node j3

    vor{j3,1}=x_vp1;...
    vor{j3,2}=y_vp1;...
end


for j1=1:N
    p1(1)=x(j1);p1(2)=y(j1);r1=r(j1);...

    xtemp=vor{j1,1};ytemp=vor{j1,2};...
    for j2=j1+1:N
      if j2~=j1
           [x_vp2,y_vp2]=visible_polygon_v2(x,y,x1,y1,j2);...%visibility_polygon-node j2  

            p2(1)=x(j2);p2(2)=y(j2);r2=r(j2);...
            [a,b,c]=line_power_voronoi_v2(p1,p2,r1,r2);...
%        null=check_for_null_vor(a,b,c,x1,y1);%elegxw an h diaxwristikh grammh einai ektos tou xwroy endiaferontos. 
        %An einai (null==1) ,tote to vor tou komvou me thn mikroterh aktina (anamesa stis r1 kai r2 einai keno 
        
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
            [in_box,out_box]=choose_halfplane_v2(a,b,c,p1,p2,box1,box2,r1,r2);...
%             [vorx1,vory1]=polybool('intersection',box1(:,1),box1(:,2),x1,y1);...
%             [vorx2,vory2]=polybool('intersection',box2(:,1),box2(:,2),x1,y1);...


            %%to temp_vorx1 tomh me to visibility_polygon (j2)  periexei to kommati pou tha petaksw apo to
            %%vor{J1,}!
            
            [temp_vorx1,temp_vory1]=polybool('intersection',out_box(:,1),out_box(:,2),xtemp,ytemp);... 
            [temp_vorx2,temp_vory2]=polybool('intersection',temp_vorx1,temp_vory1,x_vp2,y_vp2);...
%             inp=inpolygon(p1(1),p1(2),vorx1,vory1);...
%%===============vor{j1,:1}================================================
            tx=vor{j1,1};ty=vor{j1,2};...
            [xtemp,ytemp]=polybool('-',tx,ty,temp_vorx2,temp_vory2);...
            %mporei to xtemp,ytemp na exei nan dld na einai non compact
            %to cell...thelei omws epivevaiwsh..elegxw an ta duo "polugwna" 
            %einai ontws polugwna elegxontas to emvado tous 
               
             end1=find((isnan(xtemp))==1);
             if ~isempty(end1)% ean uparxei nan 
                    xtemp_v1=xtemp(1:end1-1);ytemp_v1=ytemp(1:end1-1);
                    area1=polyarea(xtemp_v1,ytemp_v1);
                        
                    xtemp_v2=xtemp(end1+1:end);ytemp_v2=ytemp(end1+1:end);
                    area2=polyarea(xtemp_v2,ytemp_v2);
                    if area1<=10^(-7) %arithmitiko sfalma
                        xtemp(1:end1)=[];ytemp(1:end1)=[];
                    elseif area2<=10^(-7)
                        xtemp(end1:end)=[];ytemp(end1:end)=[];
                    elseif area1<=10^(-7) && area2<=10^(-7)
                        xtemp=[];ytemp=[];
                    end

             end
            
            
 
            %elegxos se ola ta shmeia tou (x_temp,y_temp) gia to an anhkoun
            %sto in_box---wste na mhn fhmiourgithoun oi grammes ths
            %polybool-->
%             %=====================================================
%             s=0;...
%             index=[];...    
%             for t=1:length(xtemp)
%                 
%               if  ~isnan(xtemp(t))
% %                  plot(xtemp(t),ytemp(t),'r*')
%                 [d,x_poly,y_poly] = p_poly_dist(xtemp(t),ytemp(t),in_box(:,1), in_box(:,2));...
%                 if d>10^(-8)
%                     s=s+1;...
%                     index(s)=t;...
%                     plot(xtemp(t),ytemp(t),'g*')
%                 end
%               end 
%             end
%             if s>0
%                 s2=0;
%                 for s1=1:s
%                     xtemp(index(s1)-s2)=[];...
%                     ytemp(index(s1)-s2)=[];...
%                     s2=s2+1;
%                 end
%                 
%                 end1=find((isnan(xtemp))==1);
%                 if ~isempty(end1) && ~isempty(xtemp) % ean uparxei nan 
%                     xtemp_v1=xtemp(1:end1-1);ytemp_v1=ytemp(1:end1-1);
%                     xtemp_v1(end+1)=xtemp_v1(1);ytemp_v1(end+1)=ytemp_v1(1);
%                     
%                     xtemp_v2=xtemp(end1+1:end);ytemp_v2=ytemp(end1+1:end);
%                     xtemp_v2(end+1)=xtemp_v2(1);ytemp_v2(end+1)=ytemp_v2(1);
%                     
%                     xtemp=[xtemp_v1;xtemp_v2];ytemp=[ytemp_v1;ytemp_v2];
%                 elseif ~isempty(xtemp)
%                     xtemp(end+1)=xtemp(1);ytemp(end+1)=ytemp(1);...
%                 end
%             end
%             %=====================================================
            vor{j1,1}=xtemp;vor{j1,2}=ytemp;...
        
%     figure(1)
%     clf
%     k=1:360;
%     hold on
%     t=j1;
%     tx=vor{t,1};ty=vor{t,2};
%     plot(x1,y1,'k')
%     axis equal
%     plot(tx,ty,'g')
%    
% 
%     c2=r(t)*cosd(k)+x(t);
%     d2=r(t)*sind(k)+y(t);
%     plot(c2,d2,'r')
%     plot(x(t),y(t),'g*')
%     d
%     j2
%      pause
     
%%=========================================================================            
            


%%===============vor{j2,:}=================================================
            tx2=vor{j2,1};ty2=vor{j2,2};...
                
            [xtemp2,ytemp2]=polybool('-',tx2,ty2,xtemp,ytemp);...
            vor{j2,1}=xtemp2;vor{j2,2}=ytemp2;...
    
%     figure(1)
%     hold on
%     t=j2;
%     tx=vor{t,1};ty=vor{t,2};
%     plot(tx,ty,'b')
%    
% 
%     c2=r(t)*cosd(k)+x(t);
%     d2=r(t)*sind(k)+y(t);
%     plot(c2,d2,'r')
%     plot(x(t),y(t),'b*')
%     pause
%%=========================================================================  
           

      end 
    end
%      visVor{j1}={[xtemp ytemp]};
end


k2=1:360;

for j4=1:N
    c2=r(j4)*cosd(k2)+x(j4);...
    d2=r(j4)*sind(k2)+y(j4);...
    tx=vor{j4,1};ty=vor{j4,2};...
    if  ~isempty(tx)
        index=find(isnan(tx)==1);...
        if ~isempty(index)
            [xr1,yr1]=polybool('&',c2,d2,tx(1:index-1),ty(1:index-1));...
            [xr2,yr2]=polybool('&',c2,d2,tx(index+1:end),ty(index+1:end));...
            xr=[xr1';xr2'];yr=[yr1';yr2'];...
        else
             [xr,yr]=polybool('&',c2,d2,tx,ty);...
        end
        r_lim{j4,1}=xr;r_lim{j4,2}=yr;...
    else
        r_lim{j4,1}=[];r_lim{j4,2}=[];...
    end
end


%%svinw tis proektaseis twn tou sunorou twn vor_cell --svinw to kommati pou
%%mpainei sto vor_cell allou komvoy
% 
% for t1=1:N
%    tx1=vor{t1,1};ty1=vor{t1,2};
%    if ~isempty(tx1)
% %        length=length(tx1);
%        length=size(tx1,1);
%        s=0;
%        for t2=1:length
%            for t3=1:N
%                if t3~=t1
%                       tx3=vor{t3,1};ty3=vor{t3,2};
%                       [in,on]=inpolygon(tx1(t2),ty1(t2),tx3,ty3);
%                       if in==1 && on==0
%                           s=s+1;
%                           index(s)=t2;
% %                           tx1(t2)=[];ty1(t2)=[];
% %                           vor{t1,1}=tx1;vor{t1,2}=ty1;
% %                           length=size(tx1,1);
%                       end
%                end
%            end
%            
%        end
%        if s>0
%            for e=1:s
%                tx1(e)=[];ty1(e)=[];
%                vor{t1,1}=tx1;vor{t1,2}=ty1;
%            end
%        end
%     
%    end
% end

%%////////////////////////////////////////////////////////////////////////
% %afairw apo tis perioxes visibility-based voronoi ta mikra polygwna pou
% %bgazei h polybool
% min_area=1e-3;
% for i=1:N
%     pols={[]};
%     temp=visVor{i};
%     %arxika xwrizw to polygwno me bash ta nan
%     tempx=temp(:,1);
%     tempy=temp(:,2);
%     ind=find(isnan(tempx))';
%     ind=[0 ind length(tempx)+1];
%     for j=1:length(ind)-1;
%         pols(j)={[[tempx(ind(j)+1:ind(j+1)-1)] [tempy(ind(j)+1:ind(j+1)-1)]]};
%     end   
%     %sth analoga me to an to kathe polygwno einai mesa se kapoio allo prosthetw
%     %h afairw thn perioxh tou
%     lps=length(pols);
%     j=1;
%     while j<=lps
%         temp1=pols{j};
%         if polyarea(temp1(:,1),temp1(:,2))<min_area
%             pols(j)=[];
%             j=j-1;
%             lps=lps-1;
%         end
%         j=j+1;
%     end
%     
%     %Bazw ta kommatia ksana sth seira, kai ta xwrizw me NaN.
%     temp2=[];
%     for j=1:lps
%         temp2=[temp2;pols{j};[NaN NaN]];
%     end
%     temp2(end,:)=[];
%     visVor(i)={temp2};
%     vor{i,1}=visVor{i,1}; vor{i,2}=visVor{i,2};
% end

%%////////////////////////////////////////////////////////////////////////



%============================================

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
%============================================





return