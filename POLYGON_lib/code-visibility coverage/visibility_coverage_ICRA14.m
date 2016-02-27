% %===================================================================================================================================================================================
clear all
clc
clf 
window1=23;...
window2=23;...
%radius=8;...
R=1000;...%communication radius
N=2;...%number of robots
% r=[0.6 0.55 0.5 0.4 0.35 0.45 0.55 0.48 0.45 0.33 0.5 0.65 0.40 0.38 0.42 0.38 0.58 0.47 0.35 0.45 0.5 0.5 0.3 0.6 0.65 0.45 0.34 0.35 0.55 0.4 0.6 0.3 0.5 0.6 ]*0.9;...% 20 aktines
%r=[0.6 0.35 0.5 0.35 0.4 0.45 0.48 0.55 0.33 0.45 0.65 0.5 0.40 0.38 0.42 0.38 0.58 0.47 0.35 0.45 0.5 0.5 0.3 0.6 0.65 0.45 0.34 0.35 0.55 0.4 0.6 0.3 0.5 0.6 ]*2;...% 20 aktines
r(1:N)=1
r(N)=1

figure(1) %main figure
% axis([0 window1 0 window2])
axis equal          
hold on
%----------------------- Plot non-convex area ---------------------------------




% % 
x1=[0 0 1 1   1.5  1.5  1.8 1.8       3         3         0.8   0.8 3 3 0]';         %p-H
y1=[0 5 5 3.5 3.5  2.5  2.5 3.5     3.5         2           2     1 1 0  0]';



axis on
% axis([-6 7 -1 7])
set(gcf,'color',[1,1,1])
plot(x1,y1,'k','linewidth',1.4);...
pol=[x1(:) y1(:)];    
%--------------------------------------------------------------------------
%------------Area of convex/concave area of interest-----------------------
total_area=polyarea(x1,y1);...
%--------------------------------------------------------------------------
%----------------------Initial State---------------------------------------
disp(' Place the robots with the left mouse button ')
for g=1:1:N
 [Px,Py]=ginput(1);...
 plot(Px,Py,'b+');...
 xy(:,g) = [Px;Py];...
 x=xy(1,:);...
 y=xy(2,:);...
%r(g)=radius;...
end
 


p=[x(:) y(:)];
%--------------------------------------------------------------------------
% % %---------%save initial state-------------------------
        x0=x(1:N);...
        y0=y(1:N);...
% neigh_NxN0=visible_communication_adjacency_matrix(x0,y0,x1,y1,R,r,N);...
%-----------------------------------------------------
%[r_lim,vor]=concave_area_partitioning(x,y,x1,y1,N,r);...
%[r_lim,vor]=plot_r_limited_voronoi_v4(x,y,x1,y1,N,r);...%plot initial state

%--------------------------------------------------------------------------
%================================================================First Coordination scheme===================================================================
ts=0.001;...
umax=1;...
max_step=ts*umax ;...%max step
flag=1;...%loop in non-stop way
 h=0;...%current time-step
axis([-2.5 4 0 7]) 
%[vor,r_lim]=r_visible_voronoi_diagrams_v8(x,y,x1,y1,r,N);
%  [vor,r_lim]=visibility_power_diagrams_v2(x,y,x1,y1,N,r);
%[vor,r_lim]=power_visibility_Voronoi_v3(p,pol,r);
[vor,r_lim]=visibilityVoronoi_r_v2(p,pol,r);
plot_r_visible_voronoi_diagrams_V2(r_lim,N,x,y);
% plot_r_visible_voronoi_diagrams_V3(vor,N,x,y);
while (flag>=1);
    %tic 

     for j=1:N
        I=createI(j,N);
       
        if isempty(r_lim{j,1}) %length(r_lim{j,1})<=length(vor{j,1})%
             theta=[];
        else
             [theta,F_a,F_ax,F_ay]=gradient_motion_V8(j,r_lim,x,y,x1,y1,I,r);
             %theta(j)=theta1;
             if ~isempty(theta)
             x(j)=x(j)+ts*umax*cos(theta);
             y(j)=y(j)+ts*umax*sin(theta);
             end
        end
% % %         [theta,F_a,F_ax,F_ay]=gradient_motion_V8(j,r_lim,x,y,x1,y1,I,r);
% % %         if length(theta)>0
% % %             ux(j)=F_ax;uy(j)=F_ay;
% % %         else
% % %             ux(j)=0;uy(j)=0;
% % %         end
     end
%      if j==2
%          ux(2)=1;uy(2)=0;
%      end
%      disp('gradient:')
%      toc
%      tic
% % %      X=x(1:N);Y=y(1:N);
% % %      [tsol,xysol]=ode45(@FUNCTION_integr,[h*ts (h+1)*ts],[X(:);Y(:)],[],[ux(:);uy(:)]);
% % %      for k=1:N
% % %         [in,on]=inpolygon(xysol(end,k),xysol(end,k+N),x1,y1);
% % %         if ~or(in,on)
% % %             xysol(end,k)=X(k);
% % %             xysol(end,k+N)=Y(k);
% % %         end
% % %      end
% % %     xysol=xysol(end,:); 
% % %       
% % %     x=xysol(1:N); y=xysol(N+1:end);
        
           
        
%       x(N+1)=-100000;x(N+2)=-100000;x(N+3)=100000;x(N+4)=100000;y(N+1)=-100000;y(N+2)=100000;y(N+3)=-100000;y(N+4)=100000;...
%       disp('ode:')    
%       toc
%       tic
     %[vor,r_lim]=r_visible_voronoi_diagrams_v8(x,y,x1,y1,r,N);
     
%       [vor,r_lim]=visibility_power_diagrams_v2(x,y,x1,y1,N,r);
       p=[x(:) y(:)];
      %[vor,r_lim]=power_visibility_Voronoi_v3(p,pol,r);
     [vor,r_lim]=visibilityVoronoi_r_v2(p,pol,r);
%      disp('Tessellation:')
%      toc
%      tic
     for j=1:N
        plot(x(j),y(j),'blueo','markersize',3,'markerfacecolor','blue');
     end
     pause(0.1)
     clf, hold on, axis equal,axis off, clc
     figure(1)
     hold on
     plot(x1,y1,'k','linewidth',1.4)
     plot_r_visible_voronoi_diagrams_V2(r_lim,N,x,y) ;
%      plot_r_visible_voronoi_diagrams_V3(vor,N,x,y);
     %plot(x(j),y(j),'blueo','markersize',3,'markerfacecolor','blue');
     h=h+1;  %increase current time-step 
     sensed_area= percentage_of_sensed_area_non_compact(r_lim,total_area,N);...%evaluates current percentage of sensed area
     H(h)=sensed_area;...%save
    
     %-----save nodes' positions each time step------
     for ji=1:N 
     test_x(h,ji)=x(ji);...
     test_y(h,ji)=y(ji);...
     end
     %-----------------------------------------------
%      disp('plot:')
%      toc 
    if mod(h,100)==0
    save cdc2_new.mat
    end
end

%============================================================================================================================================================

%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

%=================================================================PlotFigures================================================================================

%---------------------plot trajectories--------------------
figure(2) 
% x0=test_x(1,:);y0=test_y(1,:);
 axis([0 6 -0.1 6])
axis equal          
hold on
axis on
plot(x1,y1,'black')
plot(x0,y0,'ro','markersize',2,'markerfacecolor','r')
plot(x(1:N),y(1:N),'go','markersize',3,'markerfacecolor','g')
plot(test_x,test_y,'k')
plot(x(1:N),y(1:N),'go','markersize',4.5,'markerfacecolor','g')
%-----------------------------------------------------------
%-----------------plot Area function H----------------------
figure
axis([0 2520 0 100])
box on
a=(N*pi*r(1)*r(1)/total_area)*100;...
if a>100
    a=100;...
end
% axis([0 h 0 a])
hold on
t=1:h;
plot(t,H,'y')
max_area(1:h)=100;...
plot(t,max_area,'red')
xlabel('Iterations')
ylabel('Coverage Performance(%)')

%-----------------------------------------------------------

%---------------------plot initial state--------------------
figure 
% axis([-0.1 3.1 -0.1 3.1])
axis equal          
hold on
axis off
plot(x1,y1,'k')
plot(x0,y0,'ro','markersize',1.5,'markerfacecolor','r')
 axis([0 window1 0 window2])
% x0(N+1)=-100000;...%Needed for Voronoi plotting
% x0(N+2)=-100000;...%...
% x0(N+3)=100000;...%...
% x0(N+4)=100000;...
% y0(N+1)=-100000;...
% y0(N+2)=100000;...
% y0(N+3)=-100000;...
% y0(N+4)=100000;...
%[r_lim,vor]=visible_r_limited_voronoi(x0,y0,x1,y1,r,N,neigh_NxN0,hops);...
%[r_lim,vor,vispol]=r_visible_voronoi_diagrams(x0,y0,x1,y1,N,r);...
p0=[x0(:) y0(:)];
  [vor,r_lim]=power_visibility_Voronoi_v3(p0,pol,r);
%   plot_r_visible_voronoi_diagrams_V2(r_lim,N,x,y);
  %[vor,r_lim]=visibility_power_diagrams_v2(x0,y0,x1,y1,N,r);
% plot_communication_graph_concave(x0,y0,N,neigh_NxN0)
for e1=1:N
        tx=(r_lim{e1,1});...
        ty=(r_lim{e1,2});... 
        plot(tx,ty,'red')
        tx=[];...
        ty=[];...    
end
%[r_lim,vor]=plot_r_limited_voronoi_v4(x0,y0,x1,y1,N,r);...
%-----------------------------------------------------------

%----------plot final state---------------------------------
figure 
 axis([0 window1 0 window2])
axis equal          
hold on
plot(x1,y1,'k')
plot(x,y,'ro','markersize',1.5,'markerfacecolor','r')
%[r_lim,vor]=visible_r_limited_voronoi(x,y,x1,y1,r,N,neigh_NxN,hops);...
%[r_lim,vor,vispol]=r_visible_voronoi_diagrams(x,y,x1,y1,N,r);...
% plot_communication_graph_concave(x,y,N,neigh_NxN)
%[r_lim,vor,vispol]=r_visible_voronoi_diagrams(x,y,x1,y1,N,r);...
%[vor,r_lim]=r_visible_voronoi_diagrams_v8(x,y,x1,y1,r,N);
   [vor,r_lim]=power_visibility_Voronoi_v3(p,pol,r);%[vor,r_lim]=visibility_power_diagrams_v2(x,y,x1,y1,N,r);
for e1=1:N
        tx=(r_lim{e1,1});...
        ty=(r_lim{e1,2});... 
        plot(tx,ty,'red')
        tx=[];...
        ty=[];...    
end
%-----------------------------------------------------------
%===================================================================================================================================================================================