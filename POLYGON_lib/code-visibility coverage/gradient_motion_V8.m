function [theta,F_a,F_ax,F_ay]=gradient_motion_V8(j,r_lim,x,y,x1,y1,r_del,r)


F_a=[];F_ax=[];F_ay=[];
xr=r_lim{j,1};%x-ccordinate of r-lim voronoi cell of node j
yr=r_lim{j,2};
%xv=cell2mat(vor{j,1});%x-ccordinate of voronoi cell of node j

%edges type c
g=1;...
% F_total=0+0*1i;...
theta=[];...
k1=360:-1:1;...
c2(k1)=r(j)*cosd(k1)+x(j);...
d2(k1)=r(j)*sind(k1)+y(j);...
[c2,d2]=poly2cw(c2,d2);...
Fe=0+0*i;
for k=1:length(xr)-1  
%     xr=floor(xr*2^13)/2^13;
%     yr=floor(yr*2^13)/2^13;
    points_x=[ xr(k) xr(k+1) ];...
    points_y=[ yr(k) yr(k+1) ];... 
    xmid=(xr(k)+xr(k+1))/2 ;...
    ymid=(yr(k)+yr(k+1))/2;...
%     xmidr=floor(xmid*2^13)/2^13;
%     ymidr=floor(ymid*2^13)/2^13;
    %[in on]=inpolygon(xmid,ymid,x1,y1);... %na mhn anhkei sto boundary kai na mhn anhkei se kanenos to r_lim
    [d,x_poly,y_poly] = p_poly_dist(xmid, ymid, x1, y1) ;%minimal distance of (xmid,ymid) from area of interset(x1,y1)
%     dist1=sqrt((xr(k)-x(j))^2+(yr(k)-y(j))^2);
%     dist2=sqrt((xr(k+1)-x(j))^2+(yr(k+1)-y(j))^2);
    [in1,on1]=inpolygon(xr(k),yr(k),c2,d2);
    [in2,on2]=inpolygon(xr(k+1),yr(k+1),c2,d2);
    if (abs(d)>sqrt(eps))  && (on1<=0 || on2<=0) %&& ((abs(dist1-r(j))>20*eps) || (abs(dist2-r(j))>20*eps))  %%na mhn anhkei sto boundary kai se tokso tou j
        s=0;
        for e=1:length(r_del)      
            xre=r_lim{r_del(e),1};%x-ccordinate of r-lim voronoi cell of node j
            yre=r_lim{r_del(e),2};
            if length(xre)>0 %prostethike giati xtupage....11/6/2013
            [d,x_poly,y_poly] = p_poly_dist(xmid, ymid, xre, yre) ;
            if abs(d)>sqrt(eps)%(on1<=0 && in1<=0) %&& (on2<=0 && in2<=0)
               s=s+1;
            end 
            else
                s=s+1;
            end
        end
        if s>=length(r_del) %na mhn anhkei se kanenos to r_lim
            th1=define_ni(points_x,points_y,xr,yr);... %define -ni for the curretn edge
                
            [A,B]=return_A(points_x,points_y,x,y,j) ;                                          %RETURN point A
            F1=estimate_integral_v8(points_x,points_y,th1,r,A,B,j,x,y);...%estimate integral for the current edge
            Fe=Fe+F1;    
            %xk=x(j)+0.5*cos(th1);yk=y(j)+0.5*sin(th1);
%           xk=x(j)+1*cos(th1);yk=y(j)+1*sin(th1);...
                
%           z(g)=1*cos(th1)+1*sin(th1)*1i;
%           z(g)=(xk-x(j))+(yk-y(j))*1i;...
%           F_total=F_total+F1;...

            %plot(xmid,ymid,'green*')
            plot(points_x,points_y,'yellow')
%             k
%             pause
%             flag='interfacing';
%             dist=sqrt((xr(k)-xr(k+1))^2+(yr(k)-yr(k+1))^2);
             
%             z(g)=(x(j)-xmid)+(y(j)-ymid)*i;...
            % %%metro(g)=abs((xr(k)-xr(k+1))+(yr(k)-yr(k+1))*i) ;...
            % %%g=g+1;...
%             pointer=k;...
        end   
    end
end
[fx,fy]=polyxpoly(c2,d2,xr,yr);...%toksa panw sto Vir
s1=1; 
F2=0+0*1i;
if length(fx)>1 && length(xr)<359 %eimaste panw sta toksa
        %epikam
        
        for q1=1:1:length(fx)-1
            F2=F2+(fx(q1)-x(j))+(fy(q1)-y(j))*1i;...
        end
%         F1(g)=F2;
%         dtheta=2*pi/360;...%1 moira
%         %F2=0+0*1i;...
%         num_of_arcs=1;
%         s1=1;
%         for q=1:length(fx)-1
%             z1=(fx(q)-x(j))+(fy(q)-y(j))*1i;theta1=angle(z1);...
%             if    q>1
%                 z2=(fx(q-1)-x(j))+(fy(q-1)-y(j))*1i;theta2=angle(z2);...
%                 dth=abs(theta1-theta2);...
%             else
%                 dth=0;...
%             end
%             if dth<=1.1*dtheta  %eimaste akoma sto idio tokso
%                 %F2=F2+z1;...
%                 thetas(s1)=theta1;...
%                 s1=s1+1;...
% %                 F1(g)=F2;%den auksanw epitides to g
%                  min_th=min(thetas);...
%                  max_th=max(thetas);
%                  if dth>0
%                       metro1(num_of_arcs)=abs((max_th-min_th)*r(j));
% %                     %plot(fx(q),fy(q),'green*')
%                  else
%                       metro1(num_of_arcs)=dtheta*r(j);
%                  end
%             else
%                 
%                 if ~isempty(thetas) %length(thetas)>0
%                 
%                     min_th=min(thetas);...
%                     max_th=max(thetas);...    
%                     %F1(g)=F2;...
%                     if length(thetas)>1
%                         num_of_arcs=num_of_arcs+1;
%                         metro1(num_of_arcs)=abs((max_th-min_th)*r(j));
%                         num_of_arcs=num_of_arcs+1;
%                         %plot(fx(q),fy(q),'green*')
%                     else
%                         metro1(num_of_arcs)=dtheta*r(j);
%                         num_of_arcs=num_of_arcs+1;
%                     end
%                     %metro(g)=(max_th-min_th)*r(j);    
%                     F2=0;...
%                    
%                     
%                     thetas=[];s1=1;
%                 else
%                      metro1(num_of_arcs)=dtheta*r(j);
%                      num_of_arcs=num_of_arcs+1;
%                 end
%             end
%             %g=g+1;
%             %metro=mikos toksou
%         end 
%          metro(g)=0;
%          for e=1:length(metro1)
%              metro(g)=metro(g)+metro1(e);...
%          end
%          g=g+1;...
end

% if abs(F_total)>0
%     theta=angle(F_total);...
% end

% 
% if g>1 || s1>1%flag=='interfacing'
%     [max1,index]=max(metro);
%     F_a=0+0*i;...
%     for t1=1:length(F1)
%         c=abs(metro(t1)/metro(index));...
%         %c=1;
%         F_a=F_a+c*F1(t1)/abs(F1(t1));...
%     end
%     theta=angle(F_a);...
%     F_ax=real(F_a);F_ay=imag(F_a);
% %     if abs(g-1)<=0
% %         theta=pi+theta;
% %     end
% end
F_a=(Fe+F2)/100;theta=angle(F_a);F_ax=real(F_a);F_ay=imag(F_a);
if abs(F_a)==0
    theta=[];
end
return

% %check for suneutheiaka shmeia sto boundary
% [L1,B1]=line_equation_v2(points_x,points_y);
%          [L2,B2]=line_equation_v2([points_x(1) x(j)],[points_y(1) y(j)]);
%          if ((abs(L2-L1)<=sqrt(eps)) && (abs(B2-B1)<=sqrt(eps)))
%             th1=define_ni(points_x,points_y,xr,yr);... %define -ni for the curretn edge
%             F1(g)=estimate_integral(points_x,points_y,th1);...%estimate integral for the current edge
%             %xk=x(j)+1*cos(th1);yk=y(j)+1*sin(th1);...
%                 
% %             z(g)=1*cos(th1)+1*sin(th1)*1i;
%   %          z(g)=(xk-x(j))+(yk-y(j))*1i;...
%              metro(g)=abs((xr(k)-xr(k+1))+(yr(k)-yr(k+1))*i) ;
%              g=g+1;...
%             %F_total=F_total+F1;...
%             plot(xmid,ymid,'blue*')
%             plot(points_x,points_y,'yellow')
%          end 

