function [vor,r_lim]=r_visible_voronoi_diagrams_v3(x,y,x1,y1,r,N)


for j=1:N
    
    
    %prepei na vrw tous komvous pou exoun kapoio koino tmhma visibility me
    %ton j kai tous apothikeuw sto  vis_nodes me thn prwth thesh na thn exei panta o j dld vis_nodes(1)=j 
    %afou tous vrw ektelw kalssiko voronoi metaksu autwn
    %---------------------------------------------------
%     [x_pol,y_pol]=visible_polygon(x,y,x1,y1,j,r);
%     
%     vis_nodes=[];
%     p=1;%deikths gia to vis_nodes
%     vis_nodes(p)=j;
%     x2(p)=x(j);y2(p)=y(j); %theseis twn komvwn pou temnontaita vis_pol tous
%      for j1=1:N
%         if j1~=j
%             [x_pol2,y_pol2]=visible_polygon(x,y,x1,y1,j1,r);...
%             [fx1,fy1]=polybool('&',x_pol,y_pol,x_pol2,y_pol2);
%             if ~isempty(fx1)
%                 p=p+1;
%                 vis_nodes(p)=j1;
%                 x2(p)=x(j1);y2(p)=y(j1);
%                 
%             end
%         end
%      end
%     x2(p+1)=-100000;x2(p+2)=-100000;x2(p+3)=100000;x2(p+4)=100000;y2(p+1)=-100000;y2(p+2)=100000;y2(p+3)=-100000;y2(p+4)=100000;...
    p=N;x2=x;y2=y;
    if p>1%ean uparxoun tetoioi komvoi
       [r_vor,vispol]=evaluate_rvoronoi_vispol_v3(r,x2,y2,p,x1,y1);...%classic voronoi diagrams
       %plot_r_visible_voronoi_diagrams(r_vor,p,x2,y2)
       xv=cell2mat(r_vor{j,1});yv=cell2mat(r_vor{j,2});...% prwth ektimhsh tou visible vor cell tou j!
       area1=polyarea(xv,yv);...    
       [x_pol,y_pol]=visible_polygon(x,y,x1,y1,j,r);
        
       [xvpol,yvpol]=polybool('&',xv,yv,x_pol,y_pol);
       [xvpol,yvpol]=poly2cw(xvpol,yvpol);
       area2=polyarea(xvpol,yvpol);
       if area2-area1>-10^(-8) 
       %plot(xv,yv,'yellow')
         del=find_r_limited_delaunay_v3(N,j,r_vor); %vriskw me poious exei ena koino edge (delaunay geitones)
         for j2=1:length(del)
           tx2=cell2mat(r_vor{del(j2),1});ty2=cell2mat(r_vor{del(j2),2});...
           %[x_pol2,y_pol2]=visible_polygon(x,y,x1,y1,j2,r);
           %[tx2,ty2]=polybool('&',tx22,ty22,x_pol2,y_pol2);
           [fx2,fy2]=find_common_edge_v2(xv,yv,tx2,ty2);... %vrsikw to koino edge      
           [x_pol2,y_pol2]=visible_polygon(x,y,x1,y1,del(j2),r);...%vis_pol enos del geitona (del(j2))
           [fx6,fy6]=polyxpoly(fx2,fy2,x_pol2,y_pol2);...%tmhma tou koinou edge pou einai orato kai stous 2
           
           %[fx6,fy6]=poly2cw(fx6,fy6);...
           fx6=fx6';fy6=fy6';...%ta thelw ola grammes
           %plot(fx6,fy6,'red')
           %periptwsh opou to fx6 fy6 periexei ena shmeio: tote to vis_pol
           %tou del(j2) temnei kapoiou allou to voronoi ...ara prepei na
           %vrw me vash to fx6,fy6 poio  einai to koino edge==>meta katalhgw sthn periptwsh line3
           
%            if (length(fx6)-1)<=0 %periptwsh opou to fx6 den vgei pleura alla shmeio
%            %prepei na vrw se poia pleura tou xv,yv anhkei to shmeio fx6,fy6
%                 [L2,B2,xa,ya]=findline_v2(xv,yv,fx6,fy6);
%                 %index= ....  elegxw an to index-1 h index+1 einai to
%                 %fx6(2)...ktl....
%            end
            
            if length(fx6)==1
                xind8=find_common_entry_v2(xv,fx6);...
                yind8=find_common_entry_v2(yv,fy6);...
                ind8=xind8(find_common_entry_v2(xind8,yind8));...
                if ~isempty(ind8)
                    fx6=[];fy6=[];
                end
            end

            
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
               %tha eleksw an to fx2,fy2 pou einai to koino edge einai visible apo ton del(j2)
                in=inpolygon(fx2(1),fy2(1),x_pol2,y_pol2);
                %an in=0 den einai visible
                if in==0
                        E=non_visible_line(fx2,fy2,xv,yv);
                        [xv1,yv1]=insert_E_in_vornoi_cell(xv,yv,fx2,fy2,E);
                        xv=xv1;yv=yv1';
%                         plot(xv,yv,'red')  
                %else:do nothing    
                end
                
                   
              
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
                     [xcand1,ycand1]=add_reflex_vertex(xp2,yp2,fx6(1),fy6(1),x1,y1); %Ak=(xcand,ycand)
                     Ak1=[xcand1 ycand1];...
                     xedge1=[xcand1 fx6(1)]; yedge1=[ycand1 fy6(1)];...
                     %epilegw to shmeio B opws orizetai sto sxedio
                     B1=choose_point_B(fx6,fy6,fx6(1),fy6(1),fx2,fy2);...
                     %vriskw thn pleura pou anhkei sto vir tou j pou arxizei sto B....prosoxh sto poia dialegw..einai 2 pleures... ftiaxnw thn BD (sxedio)
                     D1=define_BD_segment(xv,yv,fx2,fy2,B1);... %returns point D
                     %proekteinw thn BD pros to B    
                     E1=extend_BD(B1,D1,xedge1,yedge1,x1,y1);...
                     [xv1,yv1]=insert_points_in_voronoi_cell(xv,yv,Ak1,B1,E1,D1,fx6(1),fy6(1));
                      xv=xv1;yv=yv1;   
                  
                  %------gia thn deuterh korufh tou apo koinou oratou tmhmatos-----
                   [xcand2,ycand2]=add_reflex_vertex(xp2,yp2,fx6(2),fy6(2),x1,y1);...
                   Ak2=[xcand2 ycand2];...
                   xedge2=[xcand2 fx6(2)]; yedge2=[ycand2 fy6(2)];...
                    B2=choose_point_B(fx6,fy6,fx6(2),fy6(2),fx2,fy2);...
                    D2=define_BD_segment(xv,yv,fx2,fy2,B2);... 
                    E2=extend_BD(B2,D2,xedge2,yedge2,x1,y1);...
                    [xv1,yv1]=insert_points_in_voronoi_cell(xv,yv,Ak2,B2,E2,D2,fx6(2),fy6(2));
                      xv=xv1;yv=yv1;  
                      plot(xv,yv,'green')  
                     
                 elseif length(fx6)==1%flag==2 %2.2) estw mia akrh autou tou tmhmatos tautizetai me mia apo tis akres tou koinou edge (fx2,fy2)
                     [xp2,yp2]=polybool('&',tx2,ty2,x_pol2,y_pol2);... %vor tomh me vis_pol gia ton komvo del(j2)
                     [xp2,yp2]=poly2cw(xp2,yp2);...
                     %==>index=1 or 2
                     % vriskw to antistoixo reflex vertex...dld perpatw panw sto vis_pol  tou del geitona mou
                     [xcand1,ycand1]=add_reflex_vertex(xp2,yp2,fx6(1),fy6(1),x1,y1); %Ak=(xcand,ycand)
                     Ak1=[xcand1 ycand1];...
                     xedge1=[xcand1 fx6(1)]; yedge1=[ycand1 fy6(1)];...
                     %epilegw to shmeio B opws orizetai sto sxedio
                     B1=choose_point_B_v2(fx2,fy2,x_pol2,y_pol2);...
                     %vriskw thn pleura pou anhkei sto vir tou j pou arxizei sto B....prosoxh sto poia dialegw..einai 2 pleures... ftiaxnw thn BD (sxedio)
                     D1=define_BD_segment(xv,yv,fx2,fy2,B1);... %returns point D
                     %proekteinw thn BD pros to B    
                     E1=extend_BD(B1,D1,xedge1,yedge1,x1,y1);...
                     [xv1,yv1]=insert_points_in_voronoi_cell(xv,yv,Ak1,B1,E1,D1,fx6(1),fy6(1));
                     xv=xv1;yv=yv1; 
                    % plot(xv,yv,'green')  
                     
                     
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
           xv=xvpol;yv=yvpol;...
           %plot(xv,yv,'blue')    
       end
       
       
       
       
       
    else
        xv=x_pol;yv=y_pol;...
    end
 
    
    %---------------------------------------------------
    
    
    
    
    
    
    
    
    
    vor{j,1}=xv;
    vor{j,2}=yv;
    
    k=360:-1:0;...
    c2=r(j)*cosd(k)+x(j);...
    d2=r(j)*sind(k)+y(j);...
    [xr,yr]=polybool('&',xv,yv,c2,d2);...
    r_lim{j,1}=xr;
    r_lim{j,2}=yr;
    plot(xv,yv,'b') 
    
    
end

return
