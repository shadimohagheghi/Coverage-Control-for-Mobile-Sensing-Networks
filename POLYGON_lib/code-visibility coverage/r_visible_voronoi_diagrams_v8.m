function [vor,r_lim]=r_visible_voronoi_diagrams_v8(x,y,x1,y1,r,N)
 p=N;x2=x;y2=y;
 [r_vor,vispol]=evaluate_rvoronoi_vispol_v3(r,x2,y2,p,x1,y1);...%classic voronoi diagrams
vor=[];r_lim=[];
parfor j=1:N
    
    
 %if  j~=2 && j~=6 && j~=14 && j~=15
   
    if p>1%ean uparxoun tetoioi komvoi
      
       %plot_r_visible_voronoi_diagrams(r_vor,p,x2,y2)
       xv=cell2mat(r_vor{j,1});yv=cell2mat(r_vor{j,2});...% prwth ektimhsh tou visible vor cell tou j!
%        area1=polyarea(xv,yv);...
        xv_cl=xv;yv_cl=yv;
       [x_pol,y_pol]=visible_polygon(x,y,x1,y1,j,r);
        
       [xvpol,yvpol]=polybool('&',xv,yv,x_pol,y_pol);
       [xv,yv]=poly2cw(xvpol,yvpol);
%        area2=polyarea(xvpol,yvpol);
            


            
%        plot(xv,yv,'y')
         del=find_r_limited_delaunay_v3(N,j,r_vor); %vriskw me poious exei ena koino edge (delaunay geitones sumfwna me klassika voronoi)
         for j2=1:length(del)
           tx22=cell2mat(r_vor{del(j2),1});ty22=cell2mat(r_vor{del(j2),2});...
           tx2_cl=tx22;ty2_cl=ty22;
           [x_pol2,y_pol2]=visible_polygon(x,y,x1,y1,del(j2),r);
           [tx2,ty2]=polybool('&',tx22,ty22,x_pol2,y_pol2);
%            plot(tx2,ty2,'g')
           [fx5,fy5]=find_common_edge_v2(xv_cl,yv_cl,tx2_cl,ty2_cl);... %vrsikw to koino edge  (twn  klassikwn voronoi)  
                   
            % 
            if length(fx5)==4
                for t3=1:4
                    [d3,x_poly,y_poly] = p_poly_dist(fx5(t3), fy5(t3), tx22, ty22) ;
                    if abs(d3)<=(10^(-7))
                        d3=0;...
                    end
                    if d3<=0 %visible
                        save_t=t3;
                    end
                end
           
                if save_t>=3
                    fx4=[fx5(3) fx5(4)];fy4=[fy5(3) fy5(4)];...
                    
                else
                    fx4=[fx5(1) fx5(2)];fy4=[fy5(1) fy5(2)];...
                end
                fx5=fx4;fy5=fy4;
            end
            
            if length(fx5)==2
                save_t=[];
                for t3=1:2
                    [d3,x_poly,y_poly] = p_poly_dist(fx5(t3), fy5(t3), tx22, ty22) ;
                    if abs(d3)<=(10^(-7))
                        d3=0;...
                    end
                    if d3<=0 %visible
                        save_t=t3;
                    end
                end
           
                if length(save_t)>0
                    fx4=[fx5(1) fx5(2)];fy4=[fy5(1) fy5(2)];...
                    
                else
                    fx4=[];fy4=[];...
                end
                fx5=fx4;fy5=fy4;
            end
            fxe=unique(fx5);
            fye=unique(fy5);

            
            %
            if (length(fx5)==1) || (length(fxe)==1 && length(fye)==1)
                fx5=[];fy5=[];%exoun koinh mia korufh kai oxi ena edge..==>den thelw na tous thewrisw del
                fx2=[];fy2=[];
%                 disp('ll')
            else
                [fx2,fy2]=find_common_edge_v3(xv,yv,tx2,ty2);...%vrsikw to koino edge  (twn  klassikwn voronoi tomh me vispol)  
            end
           [x_pol2,y_pol2]=visible_polygon(x,y,x1,y1,del(j2),r);...%vis_pol enos del geitona (del(j2))
           [x_com_pol,y_com_pol]=polybool('&',x_pol,y_pol,x_pol2,y_pol2);...
           %---
           %[fx6,fy6]=polyxpoly(fx2,fy2,x_com_pol,y_com_pol);...%tmhma tou koinou edge pou einai orato kai stous duo(j & del(j2))
           %---
           fx6=fx2;fy6=fy2;
       
           %ALLAGH ME INPOLYGON TOU FX2 STO X_POL!!!!!!!
           fx7=[];fy7=[];...%tmhma tou edge pou einai orato ston j
           id=1;
            
            if length(fx6)==1 
                xind8=find_common_entry_v2(xvpol,fx6);...
                yind8=find_common_entry_v2(yvpol,fy6);...
                ind8=xind8(find_common_entry_v2(xind8,yind8));...
                if ~isempty(ind8)
                    fx6=[];fy6=[];
                end
            end   
            
           if length(fx6)==2 
               p2=1;
               fx8=[];fy8=[];
               for t2=1:2
                    xind9=find_common_entry_v2(xv,fx6(t2));...
                    yind9=find_common_entry_v2(yv,fy6(t2));...
                    ind9=xind9(find_common_entry_v2(xind9,yind9));...
                   
                
                    if isempty(ind9) %length()>0
                      %to petaw 
                        fx8(p2)=fx6(t2);
                        fy8(p2)=fy6(t2);
                        p2=p2+1;
                    end 
               end
                fx6=fx8;fy6=fy8;
           end
            
           
            %^^^^^^^^^^^^^^^^^^^
            flag=1;
            if ~isempty(fx6) %newcase_tes.fig
                fx10=[];fy10=[];s3=1;
                for t3=1:length(fx6)
                    xind10=find_common_entry_v2(tx2_cl,fx6(t3));...
                    yind10=find_common_entry_v2(ty2_cl,fy6(t3));...
                    ind10=xind10(find_common_entry_v2(xind10,yind10));...
                    if isempty(ind10)
                        fx10(s3)=fx6(t3);fy10(s3)=fy6(t3);
                        s3=s3+1;
                    end
                end
                fx6=fx10;fy6=fy10;
                if isempty(fx6)
                    flag=0;%mhn peirakseis to xv,yv
                end
            end
          
            %^^^^^^^^^^^^^^^^^^
            
           %3 periptwseis gia to koino edge:
           %===============================================================
           
           %---------------------------------------------------------------
           %1) to koino edge na mhn einai katholou orato sto allon
           %komvo(del(j2) :=> sto xv vriskw auta ta shmeia tou koinou edge
           %kai ta petaw
           %kwdikas gia periptwsh 1)
           %%%%xm=(fx2(1)+fx2(2))/2;ym=(fy2(1)+fy2(2))/2;
           if length(fx6)<=0 %einai eite plhrws orato eite plhrws aorato
               [xp2,yp2]=polybool('&',tx2,ty2,x_pol2,y_pol2);... %vor tomh me vis_pol gia ton komvo del(j2)
               [xp2,yp2]=poly2cw(xp2,yp2);...
               %prepei na eleksw an to fx5,fy5 einai orato ston del(j2) 
                %an einai orato den mpainw sto epomeno if
                if length(fx5)>0 && flag==1 %&& j~=10
                    [d5,x_poly,y_poly] = p_poly_dist(fx5(1), fy5(1), x_pol2, y_pol2);...
                    if abs(d5)<10^(-7)
                        d5=0;
                    end
                    if d5>0 
                        E=non_visible_line_v2(fx5,fy5,xv,yv);
                        [din,x_poly,y_poly] = p_poly_dist(E(1), E(2), x1, y1);...%elegxw an to E einai mesa sto polygwno
                        if abs(din)<10^(-7)
                            din=0;
                        end
                        if ~isempty(E) && din>0 %eksw apo to polygwno (x1,y1)
                            [E1,E2]=non_visible_line_v3(fx5,fy5,xv,yv,x1,y1);
                           [xv1,yv1]=insert_E1_E2_in_vornoi_cell_v4(xv,yv,fx5,fy5,E1,E2);
                             xv=xv1;yv=yv1;
                        end
                        if  ~isempty(E) && din<=0
                            [xv1,yv1]=insert_E_in_vornoi_cell_v3(xv,yv,fx5,fy5,E); %PROSOXH!!!!!!!!!!!
                            xv=xv1;yv=yv1;
                        
                        end
                    end
                end
%                         plot(xv,yv,'red')  
                %else:do nothing    
%                 end
%                 if length(fx7)<=0
%                     %diegrapse entelws aauto koino edge apo to xv,yv                   
%                     %++++++++++++++++++++++++++
%                     xv=xvpol;yv=yvpol; 
%                 end           
           end
           
           %---------------------------------------------------------------
           
           
           
           
           %---------------------------------------------------------------
           %2)ena tmhma tou koinou edge na einai orato:=>sto xv vriskw pou
           %einai to koino edge kai to antikathistw me to tmhma ekeino pou
           %einai orato kai stous 2 (san to r_visible_vornoi_diagrams.m)
           
%            [fx6 fy6]=poly2cw(fx6,fy6);...
%            line2(j2)=[fx6 fy6];...
%            plot(fx6,fy6,'blue')
            

            if  ~isempty(fx6) % sthn antitheth periptwsh den einai katholou orato
                %[flag,index]=choose_subcase_for_case2(fx6,fy6,fx2,fy2);
               
                 if  length(fx6)==2 %flag==1    %2.1)  oi akres tou tmhmatos autou den tautizontai me tis akres tou euthigrammou tmimatos tou koinou edge (fx2,fy2)
                     %==>index=[];
                     [xp2,yp2]=polybool('&',tx2,ty2,x_pol2,y_pol2);... %vor tomh me vis_pol gia ton komvo del(j2)
                     [xp2,yp2]=poly2cw(xp2,yp2);...
                     %prepei na perpatisw panw sto xp2,yp2 mexri na pesw panw sto reflex vertex tou 
                     
                  %-----gia thn prwth korufh tou apo koinou oratou tmhmatos-----
                  
                    % vriskw to antistoixo reflex vertex...dld perpatw panw sto vis_pol  tou del geitona mou
                     [xcand1,ycand1]=add_reflex_vertex_v2(xp2,yp2,fx6(1),fy6(1),x1,y1,xvpol,yvpol); %Ak=(xcand,ycand)
                     Ak1=[xcand1 ycand1];...
                     xedge1=[xcand1 fx6(1)]; yedge1=[ycand1 fy6(1)];...
                     %epilegw to shmeio B opws orizetai sto sxedio
                     %B1=choose_point_B(fx6,fy6,fx6(1),fy6(1),fx2,fy2);...
%                      B1=choose_point_B_v3(x_pol2,y_pol2,fx5,fy5); 
                     B1=choose_point_B(fx6,fy6,fx6(1),fy6(1),fx5,fy5);...
                     %vriskw thn pleura pou anhkei sto vir tou j pou arxizei sto B....prosoxh sto poia dialegw..einai 2 pleures... ftiaxnw thn BD (sxedio)
                     %D1=define_BD_segment(xv,yv,fx2,fy2,B1);... %returns point D
                     D1=define_BD_segment_V3(xv,yv,fx2,fy2,B1,xp2,yp2,fx5,fy5);
                     %proekteinw thn BD pros to B    
                     E1=extend_BD_v2(B1,D1,xedge1,yedge1,x1,y1);...
                     [xv1,yv1]=insert_points_in_voronoi_cell_v3(xv,yv,Ak1,B1,E1,D1,fx6(1),fy6(1));
                      xv=xv1;yv=yv1;   
                  
                  %------gia thn deuterh korufh tou apo koinou oratou tmhmatos-----
                   [xcand2,ycand2]=add_reflex_vertex_v2(xp2,yp2,fx6(2),fy6(2),x1,y1,xvpol,yvpol);...
                   Ak2=[xcand2 ycand2];...
                   xedge2=[xcand2 fx6(2)]; yedge2=[ycand2 fy6(2)];...
                    B2=choose_point_B(fx6,fy6,fx6(2),fy6(2),fx5,fy5);...
                    D2=define_BD_segment_V3(xv,yv,fx2,fy2,B2,xp2,yp2,fx5,fy5);
                    %D2=define_BD_segment(xv,yv,fx2,fy2,B2);... 
                    E2=extend_BD_v2(B2,D2,xedge2,yedge2,x1,y1);...
                    [xv1,yv1]=insert_points_in_voronoi_cell_v3(xv,yv,Ak2,B2,E2,D2,fx6(2),fy6(2));
                      xv=xv1;yv=yv1;  
                      plot(xv,yv,'green')  
                     
                 elseif length(fx6)==1%flag==2 %2.2) estw mia akrh autou tou tmhmatos tautizetai me mia apo tis akres tou koinou edge (fx2,fy2)
                     [xp2,yp2]=polybool('&',tx2,ty2,x_pol2,y_pol2);... %vor tomh me vis_pol gia ton komvo del(j2)
                     [xp2,yp2]=poly2cw(xp2,yp2);...
                     %==>index=1 or 2
                     % vriskw to antistoixo reflex vertex...dld perpatw panw sto vis_pol  tou del geitona mou
                     [xcand1,ycand1]=add_reflex_vertex_v2(xp2,yp2,fx6(1),fy6(1),x1,y1,xvpol,yvpol); %Ak=(xcand,ycand)
                     Ak1=[xcand1 ycand1];...
                     xedge1=[xcand1 fx6(1)]; yedge1=[ycand1 fy6(1)];...
                     %epilegw to shmeio B opws orizetai sto sxedio
                     B1=choose_point_B_v3(x_pol2,y_pol2,fx5,fy5);
                     %B1=choose_point_B(fx6,fy6,fx6(1),fy6(1),fx2,fy2);...
                     %vriskw thn pleura pou anhkei sto vir tou j pou arxizei sto B....prosoxh sto poia dialegw..einai 2 pleures... ftiaxnw thn BD (sxedio)
                     %D1=define_BD_segment(xv,yv,fx2,fy2,B1);... %returns point D
                     D1=define_BD_segment_V3(xv,yv,fx2,fy2,B1,xp2,yp2,fx5,fy5);
                     %proekteinw thn BD pros to B    
                     E1=extend_BD_v2(B1,D1,xedge1,yedge1,x1,y1);...
                     [xv1,yv1]=insert_points_in_voronoi_cell_v3(xv,yv,Ak1,B1,E1,D1,fx6(1),fy6(1));
                     xv=xv1;yv=yv1; 
                    % plot(xv,yv,'b')  
                     
                     
                 end
            end
           %---------------------------------------------------------------
           
           
           
           
           
           %---------------------------------------------------------------
           %3)oloklhro to koino edge na einai orato==>den to peirazw
%            
           
           
           
           %---------------------------------------------------------------
           
           
           %===============================================================
         end

       
       
       
       
       
    else
        xv=cell2mat(r_vor{j,1});yv=cell2mat(r_vor{j,2});...% prwth ektimhsh tou visible vor cell tou j!
%        area1=polyarea(xv,yv);...
        xv_cl=xv;yv_cl=yv;
       [x_pol,y_pol]=visible_polygon(x,y,x1,y1,j,r);
        
       [xvpol,yvpol]=polybool('&',xv,yv,x_pol,y_pol);
       [xv,yv]=poly2cw(xvpol,yvpol);
%         xv=x_pol;yv=y_pol;...
    end
 
    
    %---------------------------------------------------
    
    
    
    
    
    
 
    
    vor=[vor ;{[xv' yv']}];
   % vor{j,1}=xv;
    %vor{j,2}=yv;
    
    k=360:-1:0;...
    c2=r(j)*cosd(k)+x(j);...
    d2=r(j)*sind(k)+y(j);...
    [xr,yr]=polybool('&',xv,yv,c2,d2);...
    
    r_lim=[r_lim ;{[xr' yr']}];

   % r_lim{j,1}=xr;
   % r_lim{j,2}=yr;
    %plot(xv,yv,'b') 
    
% end 
end

for j=1:N
    temp1=vor{j};
    temp2=r_lim{j};
    vor1{j,1}=temp1(:,1)';
    vor1{j,2}=temp1(:,2)';
    r_lim1{j,1}=temp2(:,1)';
    r_lim1{j,2}=temp2(:,2)';
end
vor=vor1;
r_lim=r_lim1;
%%%%%%%%%%%%%%%%%%%%%%%5
% for j=2:4:6;
% [r_vor,vispol]=evaluate_rvoronoi_vispol_v3(r,x2,y2,p,x1,y1);...%classic voronoi diagrams
%        %plot_r_visible_voronoi_diagrams(r_vor,p,x2,y2)
% xv=cell2mat(r_vor{j,1});yv=cell2mat(r_vor{j,2});...% prwth ektimhsh tou visible vor cell tou j!
% %        area1=polyarea(xv,yv);...
%  xv_cl=xv;yv_cl=yv;
% [x_pol,y_pol]=visible_polygon(x,y,x1,y1,j,r);
%         
% [xvpol,yvpol]=polybool('&',xv,yv,x_pol,y_pol);
% [xv,yv]=poly2cw(xvpol,yvpol);
% vor{j,1}=xv;
%     vor{j,2}=yv;
%     
%     k=360:-1:0;...
%     c2=r(j)*cosd(k)+x(j);...
%     d2=r(j)*sind(k)+y(j);...
%     [xr,yr]=polybool('&',xv,yv,c2,d2);...
%     r_lim{j,1}=xr;
%     r_lim{j,2}=yr;
%     plot(xv,yv,'b') 
% end
% for j=14:1:15;
% [r_vor,vispol]=evaluate_rvoronoi_vispol_v3(r,x2,y2,p,x1,y1);...%classic voronoi diagrams
%        %plot_r_visible_voronoi_diagrams(r_vor,p,x2,y2)
% xv=cell2mat(r_vor{j,1});yv=cell2mat(r_vor{j,2});...% prwth ektimhsh tou visible vor cell tou j!
% %        area1=polyarea(xv,yv);...
%  xv_cl=xv;yv_cl=yv;
% [x_pol,y_pol]=visible_polygon(x,y,x1,y1,j,r);
%         
% [xvpol,yvpol]=polybool('&',xv,yv,x_pol,y_pol);
% [xv,yv]=poly2cw(xvpol,yvpol);
% vor{j,1}=xv;
%     vor{j,2}=yv;
%     
%     k=360:-1:0;...
%     c2=r(j)*cosd(k)+x(j);...
%     d2=r(j)*sind(k)+y(j);...
%     [xr,yr]=polybool('&',xv,yv,c2,d2);...
%     r_lim{j,1}=xr;
%     r_lim{j,2}=yr;
%     plot(xv,yv,'b') 
% end
% j=1;
% [r_vor,vispol]=evaluate_rvoronoi_vispol_v3(r,x2,y2,p,x1,y1);...%classic voronoi diagrams
%        %plot_r_visible_voronoi_diagrams(r_vor,p,x2,y2)
% xv=cell2mat(r_vor{j,1});yv=cell2mat(r_vor{j,2});...% prwth ektimhsh tou visible vor cell tou j!
% %        area1=polyarea(xv,yv);...
%  xv_cl=xv;yv_cl=yv;
% [x_pol,y_pol]=visible_polygon(x,y,x1,y1,j,r);
%         
% [xvpol,yvpol]=polybool('&',xv,yv,x_pol,y_pol);
% [xv,yv]=poly2cw(xvpol,yvpol);
% vor{j,1}=xv;
%     vor{j,2}=yv;
%     
%     k=360:-1:0;...
%     c2=r(j)*cosd(k)+x(j);...
%     d2=r(j)*sind(k)+y(j);...
%     [xr,yr]=polybool('&',xv,yv,c2,d2);...
%     r_lim{j,1}=xr;
%     r_lim{j,2}=yr;
%     plot(xv,yv,'b') 
return
