
%function [px, py] = combined( crs, numIterations, showPlot)
function  combined( crs, numIterations, showPlot)

%loydsAlgorithm(0.01*rand(50,1),zeros(50,1)+1/2, [0,0;0,1;1,1;1,0], 200, true)

%seed = 1;
K_prop = 0.7;
%rng(seed);

if nargin < 1   % demo mode
    showPlot = true;
    numIterations  = 30;
    xrange = 10;  %region size
    yrange = 10;
    n = 6; %number of robots  (changing the number of robots is interesting)

    Px = 0.05*mod(1:n,ceil(sqrt(n)))'*(xrange-0.5); %start the robots in a small grid
    Py = 0.05*floor((1:n)/sqrt(n))'*(yrange-0.5);
    
    Px2=Px;
    Py2=Py;
      
%     Px = rand(n,1)*(xrange-0.1); % place n  robots randomly
%     Py = rand(n,1)*(yrange-0.1);
     
    crs = [0, 0;
        0, yrange;
        xrange, yrange;
        xrange, 0;
        0,0 ];
    
    for i = 1:numel(Px)  
        while ~inpolygon(Px(i),Py(i),crs(:,1),crs(:,2)) % ensure robots are inside the boundary
            Px(i) = rand(1,1)*xrange; 
            Py(i) = rand(1,1)*yrange;
        end
    end
    %copy real generator positions to virtual positions.
    Vx = Px;
    Vy = Py;
else
    xrange = max(crs(:,1));
    yrange = max(crs(:,2));
    n = numel(Px); %number of robots  
end

%%%%%%%%%%%%%%%%%%%%%%%% VISUALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if showPlot
    figure(1);
    %hold on
    %figure(2);
    verCellHandle = zeros(n,1);
    cellColors = hsv(n);
    for i = 1:numel(Px) % color according to
        verCellHandle(i)  = patch(Px(i),Py(i),cellColors(i,:)); % use color i  -- no robot assigned yet
        hold on
    end
    pathHandle = zeros(n,1);    
    %numHandle = zeros(n,1);    
    for i = 1:numel(Px) % color according to
        %pathHandle(i)  = plot(Px(i),Py(i),'-','color',cellColors(i,:)*.8);
        %numHandle(i) = text(Px(i),Py(i),num2str(i));
    end
    
    goalHandle = plot(Px,Py,'+','linewidth',2, 'color','black');
    %currHandle = plot(Px,Py,'o','linewidth',2, 'color','black');
    
    goalHandle2 = plot(Px2,Py2,'*','linewidth',2, 'color','black');
    obs = zeros(1,4,2);
     obs(1,:,:) = [2 2; 8 2; 8 8; 2 8];
%     plot(obs1(:,1),obs1(:,2),'ro');%plot red circles for visualization
%     
%     patch(obs1(:,1),obs1(:,2),'g'); %plot obstacle
%     
%     %similarly for the rest of the obstacles
%     
%     obs2 = [6 2; 7 2; 7 3; 6 3];
%     plot(obs2(:,1),obs2(:,2), 'ro');
%     
%     patch(obs2(:,1),obs2(:,2),'g');
% 
%     
%     obs3 = [3 6; 4 6; 4 7; 3 7];
%     plot(obs3(:,1),obs3(:,2),'ro');
% 
%     patch(obs3(:,1),obs3(:,2),'g');
%     obs4= [6 6; 7 6; 7 7; 6 7];
%     plot(obs4(:,1),obs4(:,2),'ro');
%     
%     patch(obs4(:,1),obs4(:,2),'g');
%     
     
    titleHandle = title(['o = Robots, + = Goals, Iteration ', num2str(0)]);
    hold on;
end
%%%%%%%%%%%%%%%%%%%%%%%% END VISUALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
DISPLACEMENT = zeros(numIterations,1);
MAX_DISPLACEMENT = 0.5;

% Iteratively Apply LLYOD's Algorithm
for counter = 1:numIterations
    
    %loydsAlgorithm_nonuniform(i,Px,Py, crs, numIterations, showPlot)
    %[v,c]=VoronoiLimit(Px,Py, crs, false);
    [v,c]=VoronoiBounded(Vx,Vy, crs);
    
%     if showPlot
%         set(currHandle,'XData',Px,'YData',Py);%plot current position
%         for i = 1:numel(Px) % color according to
%             xD = [get(pathHandle(i),'XData'),Px(i)];
%             yD = [get(pathHandle(i),'YData'),Py(i)];
%             set(pathHandle(i),'XData',xD,'YData',yD);%plot path position
%             set(numHandle(i),'Position',[ Px(i),Py(i)]);
%         end 
%     end
   
    for i = 1:numel(c) %calculate the centroid of each cell
        % Changed for non_uniform case
        %[cx,cy] = PolyCentroid(v(c{i},1),v(c{i},2));
        
        [cx,cy] = PolyCentroidNonuniformDensity(v(c{i},1),v(c{i},2),counter);
        
        cx = min(xrange,max(0, cx));
        cy = min(yrange,max(0, cy));
        
        if ~isnan(cx) && inpolygon(cx,cy,crs(:,1),crs(:,2))
                Vx(i) = Vx(i) + -K_prop*(Vx(i) - cx);
                Vy(i) = Vy(i) + -K_prop*(Vy(i) - cy);
                
        end
        
        %Update real positions
        i;
        Px2(i)=cx;
        Py2(i)=cy;
        [Px(i) Py(i)] = tangentBug( Px(i), Py(i), cx, cy, obs);
        
        %[Px(i) Py(i)] = tangentBug(cx, cy, Px(i), Py(i), obs);
    end
       
       if showPlot
        
        for i = 1:numel(c) % update Voronoi cells
            %verCellHandle(i)
            set(verCellHandle(i), 'XData',v(c{i},1),'YData',v(c{i},2));
        end

        set(titleHandle,'string',['o = Robots, + = Goals, Iteration ', num2str(counter,'%3d')]);
        set(goalHandle,'XData',Px,'YData',Py);%plot goal position
        set(goalHandle,'XData',Px,'YData',Py)
        
        set(goalHandle2,'XData',Px2,'YData',Py2);%plot goal position
        set(goalHandle2,'XData',Px2,'YData',Py2)
        
        axis equal
        axis([0,xrange,0,yrange]);
        drawnow
%         if mod(counter,50) ==0
%             pause
%             %pause(0.1)
%         end
       end
       

end

% V = 1:numIterations;
% figure (2);
% DISPLACEMENT_LOG = log(DISPLACEMENT);
% 
% f = fit(V',DISPLACEMENT_LOG(:,1), 'poly1');
% plot(V,DISPLACEMENT_LOG);
% plot(f,V,DISPLACEMENT_LOG);

% function [Cx,Cy] = PolyCentroid(X,Y)
% % The centroid of a non-self-intersecting closed polygon defined by n vertices (x0,y0), (x1,y1), ..., (xn?1,yn?1) is the point (Cx, Cy), where
% % In these formulas, the vertices are assumed to be numbered in order of their occurrence along the polygon's perimeter, and the vertex ( xn, yn ) is assumed to be the same as ( x0, y0 ). Note that if the points are numbered in clockwise order the area A, computed as above, will have a negative sign; but the centroid coordinates will be correct even in this case.http://en.wikipedia.org/wiki/Centroid
% % A = polyarea(X,Y)
% 
% Xa = [X(2:end);X(1)];
% Ya = [Y(2:end);Y(1)];
% 
% A = 1/2*sum(X.*Ya-Xa.*Y); %signed area of the polygon
% 
% Cx = (1/(6*A)*sum((X + Xa).*(X.*Ya-Xa.*Y)));
% Cy = (1/(6*A)*sum((Y + Ya).*(X.*Ya-Xa.*Y)));

end

function a = triangle_area(P1,P2,P3)
    e12 = sqrt(sum((P1-P2) .^ 2));
    e23 = sqrt(sum((P2-P3) .^ 2));
    e31 = sqrt(sum((P1-P3) .^ 2));
    s = (e12 + e23 + e31)/2;
    a = sqrt(s*(s-e12)*(s-e23)*(s-e31));    
end

%A toy density function - Gaussian density around center point (cx,cy)
function r = density(x,y, i)
   %r = exp(-(x-0.02*cos(i))*(x-0.02*cos(i)) + -(y-0.02*sin(i))*(y-0.02*sin(i)));
   %r = exp(-(x-5*cos(2*pi*i/180)-5)*(x-5*cos(2*pi*i/180)-5) + -(y-5*sin(2*pi*i/180)-5)*(y-5*sin(2*pi*i/180)-5));
   r = exp(-((x-5)*(x-5)) + -((y-5)*(y-5))); %+ exp(-((x-7.5)*(x-7.5)) + -((y-7.5)*(y-7.5)));
   %r = exp(-(abs(y-5)));
   %r = 1;
end

function [Cx,Cy] = PolyCentroidNonuniformDensity(X,Y, i)
    CELL_SAMPLES = 500;
    %Triangulate points - returns a list of indices
    n = size(X,1)-1;
    x_r = fliplr(X);
    y_r = fliplr(Y);
    
    triangles = polygon_triangulate(n, x_r(1:n), y_r(1:n));
  
    j = 1;
    
    %Array of areas of each triangle in the polygon
    tri_areas(1:n-2,1) = 0;
    
    %Calculate triangle areas & convert indices to points
    while j <= n-2
        %Grab indices of jth triangle
        i1 = triangles(1,j);
        i2 = triangles(2,j);
        i3 = triangles(3,j);
        
        %GRab points of triangle
        P1 = [X(i1) Y(i1)];
        P2 = [X(i2) Y(i2)];
        P3 = [X(i3) Y(i3)];
        
        tri_areas(j,1) = triangle_area(P1,P2,P3); 
        
        j = j + 1;
    end
    
    total_area = sum(tri_areas);
    j = 1;
    
    %Index of current sample, updated in loop
    current_sample = 1;
    
    %Storage array for random points on polygon
    random_points = zeros(CELL_SAMPLES,2);
    
    while j <= n-2
        % Calculate number of samples to make in triangle based on
        % fractional area of triangle in voronoi cell
        sample_num_j = floor( (CELL_SAMPLES * tri_areas(j,1)) / (total_area));
        
        %Take random samples 
        sample_points = sample_rand_triangle([X(triangles(1,j)) Y(triangles(1,j)) ] , ...
            [X(triangles(2,j)) Y(triangles(2,j)) ] , ...
            [X(triangles(3,j)) Y(triangles(3,j)) ],sample_num_j);
        %assert(current_sample + sample_num_j - 1 <= CELL_SAMPLES);
        
        %Save the generated points - there are sample_num_j such points
        index_end = current_sample + sample_num_j - 1;
        random_points(current_sample:index_end,:) = sample_points;
        
        %Update index of current sample
        current_sample = current_sample + sample_num_j ;
        j = j + 1;
    end
    
    %Calculate centroid based on sample points in triangle based on Monte
    %Carlo Integration
    j = 1;
    numerator_vec_sum = [0 0];
    denominator_sum = 0;
    while j < current_sample
        D = density(random_points(j,1),random_points(j,2), i);
        numerator_vec_sum = numerator_vec_sum + random_points(j,:)*D;
        denominator_sum = denominator_sum + D;
        j = j + 1;
    end
    
    C_v = numerator_vec_sum / denominator_sum;
    Cx = C_v(1);
    Cy = C_v(2);
end

%Use Osada et. al [02] method for random sampling from triangle
function P = sample_rand_triangle(P1,P2,P3, N)
    
    Bounds(1:N,1) = 0;
    Bounds(1:N,2) = 1;
    
    R1 = unifrnd(Bounds(1:N,1),Bounds(1:N,2));
    R2 = unifrnd(Bounds(1:N,1),Bounds(1:N,2));
    
    %Points array
    P(1:N,1:2) = 0;
    i = 1;
    while i <= N
        %generate a point on the triangle from random variables r1(i),
        %r2(i)
        P(i,1:2) = (1-sqrt(R1(i)))*P1 + sqrt(R1(i))*(1-R2(i))*P2 + sqrt(R1(i))*R2(i)*P3;
        i = i + 1;
    end
end

function [V,C]=VoronoiBounded(x,y, crs)

bnd=[min(x) max(x) min(y) max(y)]; %data bounds
if nargin < 3
    crs=double([bnd(1) bnd(4);bnd(2) bnd(4);bnd(2) bnd(3);bnd(1) bnd(3);bnd(1) bnd(4)]);
end

rgx = max(crs(:,1))-min(crs(:,1));
rgy = max(crs(:,2))-min(crs(:,2));
rg = max(rgx,rgy);
midx = (max(crs(:,1))+min(crs(:,1)))/2;
midy = (max(crs(:,2))+min(crs(:,2)))/2;

% add 4 additional edges
xA = [x; midx + [0;0;-5*rg;+5*rg]];
yA = [y; midy + [-5*rg;+5*rg;0;0]];

[vi,ci]=voronoin([xA,yA]);

% remove the last 4 cells
C = ci(1:end-4);
V = vi;
% use Polybool to crop the cells
%Polybool for restriction of polygons to domain.

for ij=1:length(C)
        % first convert the contour coordinate to clockwise order:
        [X2, Y2] = poly2cw(V(C{ij},1),V(C{ij},2));
        [xb, yb] = polybool('intersection',crs(:,1),crs(:,2),X2,Y2);
        ix=nan(1,length(xb));
        for il=1:length(xb)
            if any(V(:,1)==xb(il)) && any(V(:,2)==yb(il))
                ix1=find(V(:,1)==xb(il));
                ix2=find(V(:,2)==yb(il));
                for ib=1:length(ix1)
                    if any(ix1(ib)==ix2)
                        ix(il)=ix1(ib);
                    end
                end
                if isnan(ix(il))==1
                    lv=length(V);
                    V(lv+1,1)=xb(il);
                    V(lv+1,2)=yb(il);
                    ix(il)=lv+1;
                end
            else
                lv=length(V);
                V(lv+1,1)=xb(il);
                V(lv+1,2)=yb(il);
                ix(il)=lv+1;
            end
        end
        C{ij}=ix;
   
end
end

%function for tangent bug. have to tweak the sensor range and the distance
%moved by the robot in a single step to get good performance. can enter
%nonconvex obstacles by a combination of convex obstacles. but will have to
%tweak both said parameters otherwise may terminate incorrectly.

%main fuction
function [px,py] = tangentBug (startx, starty, endx, endy, obs)

    %figure;             %initialize the plotting window
    axis ([0 10 0 10]);
    
    %give all variables global scope so that other functions may see them    

    global obstacle;  % N x (P,2) matrix for N obstacles, each with P vertices

                        
    global d_set;       %matrix to store the vertices of discontinuities in scan
    global scan;        %the array to save current scan of the environment 
    global range;       %the maximum range of the sonar array (for saturated distance function
    global r_res;       %resolution of the sonar
    global theta_res;   %angular resolution of the sonar
    global r_step;      %the distance the robot moves during a single step
    global robot;       %the location of the robot
    global start;       %the start point
    global goal;        %the goal point
    
    global d_planner; %to remember direction of heuristic increase or decrease
                      %contains the 2 most recent values of the heuristic
                      %distance
    global r_planner; %to remember direction of robot motion. contains the
                      %most recent 2 locations of the robot. needed to
                      %decide direction of wall following
                      
    global dmin;      %as defined in slides. used for termination of wall following
    global dreach;    %also known as dleave.  
    
    global mins;      %the minimum value of the current scan array. used to 
                      %mantain a safe distance from the obstacles
                      %mins(1) stores that value and mins(2) stores the
                      %index of the scan array from which that value was
                      %taken

    global robo_hist; %keeps record of the robot path 
                      %will be used to terminate the algorithm
                      
    global wcount;    %flags used for programming purposes
    global d_theta;
    global wflag;
    global ncount;
    global exit_flag;

    obstacle = obs;
    scan = zeros(1,12);
    d_planner = zeros(1,2) + 100;
    r_planner = zeros(2,2);
    d_set = [];
    range = 2;
    r_res = 0.1;
    theta_res = 4*pi/180;
    r_step = 0.3;
    dmin=0;
    dreach=0;
    d_theta=0;
    wflag=1;
    mins=zeros(1,2);
    ncount=0;
    
    exit_flag=0; %this flag will be set when goal cant be reached
    
    f = text(0.5,-0.75,'specify vertices of 4 obstacles (convex). for each obstacle give 4 vertices');
    figure(2)
    set(f,'String','enter start point');
    set(f,'Position',[4 -0.75]);
    start = [startx starty];      %get start point
    robot = start;
    r_planner(2,:) = robot;
    %plot(start(1),start(2),'o','linewidth',2, 'color','black');;
    
    set(f,'String','enter goal point');
    %goal = ginput(1);       %get goal point
    goal=[endx endy];
    hold on;
    
    %project the target into the region
    for i=1:size(obstacle,1)
        if (~inpolygon(goal(1), goal(2), obstacle(i,:,1), obs(i,:,2)))
            continue;
        else
            %find the point on the boundary of the obstacle that minimizes
            %the perpendicular bisector
            %aka project
            min_point = [-1 -1];
            min_dist = Inf;
            for j=1:size(obstacle(i,:,:),2)
                %for vertex i, calculate the line between (i,i+1)
                line1 = [obstacle(i,j,1) obstacle(i,j,2)];
                if (j == size(obstacle(i,:,:),2))
                    line2 = [obstacle(i,1,1) obstacle(i,1,2)];
                else
                    line2 = [obstacle(i,j+1,1) obstacle(i,j+1,2)];
                end
                
                int_point = proj(goal, [line1 , line2]);
                dist = euc_dist(int_point',goal);
                if (dist < min_dist)
                    min_dist = dist;
                    min_point = int_point;
                end
            end
            %assert((min_point == [-1;-1]) ~= [ 1 1]);
            %Set goal to projected point
            goal = min_point';
            break;
        end
    end
 
    
    plot(goal(1),goal(2),'m');
    %plot(startx, starty,'.','linewidth',3, 'color','black')
    %plot(centroidsX,centroidsY,'+','linewidth',2, 'color','black')
    pause(0.01);
    
    get_scan();             %scan the environment
    get_discontinuities();  %get discontinuities in the scan
    %plot(start(1),start(2),'o','linewidth',2, 'color','black');
    %plot(centroidsX,centroidsY,'+','linewidth',2, 'color','black')
    %plot(startX, startY,'.','linewidth',3, 'color','black')
    plot(goal(1),goal(2),'+','linewidth',2, 'color','black');
    
    pause(0.01);             %time delay

    %remain inside loop until goal is reached
    while euc_dist(robot,goal)>0.2 && exit_flag==0
        cla;                %clear the plotting window
        
        if mins(1)<0.5        %if the robot is too close to an obstacle, then
                              %do wall following irrespective of all behaviours 
          	%f = text(4,-0.75,'doing wall following');
            temp_step=r_step; 
            r_step=0.4;       %do it with an increased setp;  
            wall_follow();    %call the wall following function
            r_step=temp_step; %restore the step
        
        elseif size(d_set)==0   %if no discontinuities, move towards goal
            move_towards_goal();
            %f = text(4,-0.75,'moving towards goal');
            wcount=0;
            wflag=1;
            
        elseif (d_planner(2)>d_planner(1)) && (wflag==1)
                                        %if heuristic distance is not
                                        %increasing, then do wall following
                                        
          	%f = text(4,-0.75,'doing wall following');
            if dmin<=dreach             %do while dleave does not go below dmin
                temp_step=r_step;       %doing it with an increased step
                r_step=0.4;
                wall_follow();
                wcount = wcount+1;      
                r_step = temp_step;
            end
            
        else
            move_towards_discontinuity();   %move towards discontinuity
            %f = text(4,-0.75,'moving towards discontinuity');
                                            %update the planner to mantain
                                            %the direction of increase of
                                            %the heuristic distance
            d_planner(1) = d_planner(2);
            dist_r = euc_dist(robot,[d_set(1,1) d_set(1,2)]); %distance from robot to discontinuity
            dist_d = euc_dist(goal,[d_set(1,1) d_set(1,2)]);  %distance from discontinuity to goal
            d_planner(2) = dist_d + dist_r;            
            wcount=0;
            wflag=1;
        end
        
        hold on;
        plot_world();                       %plot all obstacles
        get_scan();                         %get scan,discontinuities and plot start,goal and robot
        get_discontinuities();
        %plot(start(1),start(2),'o','linewidth',2, 'color','black');
        plot(goal(1),goal(2),'+','linewidth',2, 'color','black');
        %plot(centroidsX,centroidsY,'+','linewidth',2, 'color','black');        
        %plot(startx, starty,'.','linewidth',3, 'color','black');
        plot(robot(1),robot(2),'.','linewidth',5, 'color','g');

        %update the planner to mantain the vector of robot direction
        r_planner(1,:) = r_planner(2,:);
        r_planner(2,:) = robot;
        pause(0.01);                         %pause needed so that the data is plotted properly.
        %HACK
        break;
    end
    if exit_flag==0
        %set(f,'String','reached goal successfully');
        %Unsure about what i deleted.
        %Px = startX;
        %Py = startY;
    else
        %set(f,'String','no path to goal exists');
    end
    hold off;
    px = robot(1);
    py = robot(2);

end

function PT2 = proj(point, line)

vx = line(:, 3);
vy = line(:, 4);

dx = point(:,1) - line(:,1);
dy = point(:,2) - line(:,2);

px = -1*point(1)*(line(3) - line(1)) - point(2)*(line(4) - line(2));
py = -1*line(2)*(line(3) - line(1)) + line(1)*(line(4) - line(2));


rhs =-1*[px; py];
lhs(1,1) = (line(3) -line(1));
lhs(1,2) = (line(4) - line(2));
lhs(2,1) = (line(2) - line(4));
lhs(2,2) = (line(3)-line(1));

PT2 = lhs\rhs;
end

function g = get_line(point1,point2)

    line = zeros(1,3);
    line(1) = point2(2) - point1(2);
    line(2) = point1(1) - point2(1);
    line(3) = point1(2) * ( point2(1) - point1(1) ) - point1(1) * ( point2(2) - point1(2) );
    g =line;

end

function plot_world()
    global obstacle;
    hold on;
    %simple patch commands
    %TODO: check array size
    for i =1:size(obstacle,1)
        patch(obstacle(i,:,1), obstacle(i,:,2) ,'g');
    end
    hold on; 
end


%this function returns 1 if a point lies inside an obstacle and 0 if it
%lies outside
function g = check_for_obstacles(point)
    global obstacle;
    g = 0;
    for i = 1:size(obstacle,1)
        if inpolygon(point(1),point(2),obstacle(i,:,1),obstacle(i,:,2))
            g = 1;
            break;
        end
    end
    
end

%this function plots the current scan of the environment and stores it in
%an array
function get_scan()

    global scan;
    global range;
    global r_res;
    global theta_res;
    global robot;
    global mins;

    beams = 2*pi/theta_res; %total number of beams calculated by set resolution
    scan = zeros(1,beams);  %define the array
    scan = scan+range;      %initialze all ranges to be of max range
    n=0;                    %counter used for mantaining the index of the array
    mins(1)=4;
    
    for t = 0:theta_res:2*pi - theta_res %traverse all values of r and theta
        n=n+1;
        for r = 0:r_res:range
            
            x = robot(1)+r*cos(t); %get point in cartesian cordinate
            y = robot(2)+r*sin(t);
            %if point is in obstacle, set the range of that sonar beam to
            %the current value of r
            if(check_for_obstacles([x y])==1)
                
                scan(n) = r-r_res;
                if scan(n)<mins(1)  %lookfor the minimum value of the array
                    mins(1)=scan(n);
                    mins(2)=n;
                end
                break;
            end
            %plot(x,y,'b.'); %plot the points in freespace. good for visualizing
        end
    end

end

function get_discontinuities()

    global scan;
    global range;
    global robot;
    global d_set;
    global goal;

    %first determine size of scan array and initialize the discontinuity
    %set
    s = size(scan,2);
    d_set = [];
    n = 0;
    %traverse the whole scan array
    for a=2:1:s
      
        %search for discontinuities
        %a discontinuity is defined in this code as the point at which 
        %is not at maximum range of the sonar but which has a neighbour
        %on the sonar array which returns maximum range
        if ( scan(a) ~= scan(a-1) ) && ( scan(a)==range || scan(a-1)==range)
            if scan(a)==range
                r = scan(a-1);
                t = (a-2)*2*pi/s;
                d = a-1;
            else
                r = scan(a);
                t = (a-1)*2*pi/s;
                d = a;
            end
            %determine wether the discontinuity is nearer to the goal then
            %the robot's current position. if not then there is no use in
            %saving it
            x=robot(1)+r*cos(t);
            y=robot(2)+r*sin(t);
            dist_r = euc_dist(robot,goal);
            dist_d = euc_dist([x y],goal);
            %if yes then include it in the discontinuity set
            if dist_d<dist_r
                n=n+1;
                d_set(n,:) = [x y d];
            end
            %display all discontinuity points
            %plot(x,y,'r*');
        end
        
    end
    %sort the discontinuities according to the heuristic distance
    %so that the first element will always be the discontinuity the robot
    %has to follow
    %sorting has been done by insertion sort (not very efficient)
    %the parameter used for sorting is the heuristic distance of course
    if size(d_set)~=0
        for a=2:size(d_set)
            insert = d_set(a,:);
            dist_r = euc_dist(robot,[d_set(a,1) d_set(a,2)]);
            dist_d = euc_dist(goal,[d_set(a,1) d_set(a,2)]);
            dist_h = dist_r+dist_d;
            move=a;
            dist_r = euc_dist(robot,[d_set(move-1,1) d_set(move-1,2)]);
            dist_d = euc_dist(goal,[d_set(move-1,1) d_set(move-1,2)]);
            compare = dist_r+dist_d;
            while move>1 && compare>dist_h
                d_set(move,:) = d_set(move-1,:);
                dist_r = euc_dist(robot,[d_set(move-1,1) d_set(move-1,2)]);
                dist_d = euc_dist(goal,[d_set(move-1,1) d_set(move-1,2)]);
                compare = dist_r+dist_d;
                move=move-1;
            end
            d_set(move,:) = insert;
        end
    end
end

%function increases the coordinates of the robot in direction of the goal
function move_towards_goal()
    global goal;
    global robot;
    global r_step;
    global r_planner;
    
    %get the angle on which to move the robot
    t = atan2(goal(2) - robot(2),goal(1) - robot(1));
    if t<0
        t=t+2*pi;
    end
    %update the position
    robot(1) = robot(1) + r_step*cos(t);
    robot(2) = robot(2) + r_step*sin(t);
    
end

%function to increase the coordinates of the robot in direction of the
%most favourable discontinuity
function move_towards_discontinuity()
    global d_set;
    global robot;
    global r_step;
    global r_planner;
    global mins;
    
    %calculate direction on which to move robot
    t = atan2(d_set(1,2) - robot(2),d_set(1,1) - robot(1));
    if t<0
        t=t+2*pi;
    end

    if mins(1)<0.5
        wall_follow(); %just as a safety measure. not really required here
    else
        %update the position
        robot(1) = robot(1) + r_step*cos(t);
        robot(2) = robot(2) + r_step*sin(t);
    end
end

%function to increase the coordinates of the robot in general direction of
%wall following
function wall_follow()
    global scan;
    global theta_res;
    global r_step;
    global robot;
    global r_planner;
    global goal;
    global wcount;
    global wflag;
    global ncount;
    global robo_hist;
    
    %first got to set value of dmin
    s = size(scan,2);   
    r = scan(1);
    t = (1-1)*2*pi/s;
    x=robot(1)+r*cos(t);
    y=robot(2)+r*sin(t);
    dmin = euc_dist(goal,[x y]);
    %remember that dleave and dmin start out the same
    dreach = dmin;

    %if wall following is being executed for first time, update dmin
    %not really following the idea of the algorithm here (dmin should be
    %updated constantly),but it is being done so, indirectly
    if wcount==1

        for a=1:s
            r = scan(a);
            t = (a-1)*2*pi/s;
            x=robot(1)+r*cos(t);
            y=robot(2)+r*sin(t);
            dist_d = euc_dist(goal,[x y]);
            if dist_d<dmin                  %update dmin
                dmin=dist_d;
            end
        end
        
   end

    smallest=1; %index of the smallest range of scan. could also have used mins instead
    
    %now update dreach (dleave)
    for a=1:s
        if scan(a)<scan(smallest)
            smallest=a;
        end
        r = scan(a);
        t = (a-1)*2*pi/s;
        x=robot(1)+r*cos(t);
        y=robot(2)+r*sin(t);
        dist_d = euc_dist([x y],goal);
        if dist_d<dreach                %update dreach
            dreach=dist_d;
        end
    end
    
    if dreach<dmin  %terminate wall following by setting flag
        wflag=0;    %look in the main function to see how this works    
    end
    
    %now to calculate the direction in which to move the robot. the
    %implementation is as covered in class. also we have to choose between
    %the two possible direction for wall following which will be in the
    %direction the robot was already moving in
    
    t = theta_res*(smallest-1); %direction of normal to obstacle
    %direction of robot motion calculated from the planner
    r_theta = atan2((r_planner(2,2)-r_planner(1,2)),(r_planner(2,1)-r_planner(1,1)));
    %scale theta. because the range of atan2 is from -pi to +pi
    if r_theta<0
        r_theta=r_theta+2*pi;
    end
    
    %take dot product of possible tangent direction and robot motion vector
    if dot([cos(r_theta) sin(r_theta)],[cos(t+pi/2) sin(t+pi/2)])>=0;
        g=t+pi/2+pi/24;
    else                %pi/24 added as a factor of safety
       g=t-pi/2+pi/24;
    end

    %again just for safety
    if(check_for_obstacles([(robot(1)+r_step*cos(g)) (robot(2)+r_step*sin(g))])==1)
        g=g+pi;
    end
    
    %update the robot coordinates
    robot(1) = robot(1) + r_step*cos(g);
    robot(2) = robot(2) + r_step*sin(g);
   
    ncount=ncount+1;
    robo_hist(ncount,:) = [robot(1) robot(2)];
    check_for_cycle();
end

%simple function to calculate euclidean distance between two points in 2D
function g = euc_dist(x,y)
    g=sum((x-y).^2).^0.5;
end

%checks wether the current robot postion has been visited before or not
function check_for_cycle()
    %search wether the robot has visited this position before
    %if yes then it is moving in a cycle and no path to the goal exists
    global robot;
    global exit_flag;
    global robo_hist;
    global ncount;
    for a=1:ncount-1;
        if euc_dist([robo_hist(a,1) robo_hist(a,2)],[robot(1) robot(2)])<0.025
            exit_flag=1;
        end
    end
end
