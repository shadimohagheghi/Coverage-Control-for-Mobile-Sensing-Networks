function agent_locations = loydsAlgorithm_nonuniform(numIterations,showPlot,num_agents,obstacles,seed,control_gain)

%loydsAlgorithm(0.01*rand(50,1),zeros(50,1)+1/2, [0,0;0,1;1,1;1,0], 200, true)

close all
K_prop = control_gain;
rng(seed);

xrange = 30;  %region size
yrange = 30;
n = num_agents; %number of robots  (changing the number of robots is interesting)
Px = zeros(n,1);
Py = zeros(n,1);
agent_locations = zeros(numIterations,n,2);


%Bounding box setup
crs = [ 0, 0;
    0, yrange;
    xrange, yrange;
    xrange, 0];

%Setup aegnt locations randomly
for i = 1:n
    valid_location = 0;
    while (valid_location == 0)
        %Setup location as valid (hypothesis)
        Px(i) = rand()*xrange; 
        Py(i) = rand()*yrange;
        
        valid_location = 1;
        %Test for all obstacles
        for ob =1:size(obstacles,1)
            if (inpolygon(Px(i),Py(i),obstacles(ob,:,1), obstacles(ob,:,2)))
                valid_location = 0;
                break;
            end
        end

    end
end


%%%%%%%%%%%%%%%%%%%%%%%% VISUALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if showPlot
    verCellHandle = zeros(n,1);
    cellColors = hsv(n + size(obstacles,1));
    for i = 1:numel(Px) % color according to
        verCellHandle(i)  = patch(Px(i),Py(i),cellColors(i,:)); % use color i  -- no robot assigned yet
        hold on
    end
    pathHandle = zeros(n,1);    
    numHandle = zeros(n,1);    
    for i = 1:numel(Px) % color according to
        pathHandle(i)  = plot(Px(i),Py(i),'-','color',cellColors(i,:)*.8);
        numHandle(i) = text(Px(i),Py(i),num2str(i));
    end
        
    for j = 1:size(obstacles,1)
        obstacleHandle(j) = patch(obstacles(j,:,1),obstacles(j,:,2),cellColors(n+1,:));
    end
    goalHandle = plot(Px,Py,'+','linewidth',2, 'color','black');
    currHandle = plot(Px,Py,'o','linewidth',2, 'color','black');
    titleHandle = title(['o = Robots, + = Goals, Iteration ', num2str(0)]);
end
%%%%%%%%%%%%%%%%%%%%%%%% END VISUALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Iteratively Apply LLYOD's Algorithm
for counter = 1:numIterations

    %[v,c]=VoronoiLimit(Px,Py, crs, false);
    [v,c]=VoronoiBounded(Px,Py, crs);
    
    if showPlot
        set(currHandle,'XData',Px,'YData',Py);%plot current position
        for i = 1:numel(Px) % color according to
            xD = [get(pathHandle(i),'XData'),Px(i)];
            yD = [get(pathHandle(i),'YData'),Py(i)];
            set(pathHandle(i),'XData',xD,'YData',yD);%plot path position
            set(numHandle(i),'Position',[ Px(i),Py(i)]);
        end 
    end
    
                
    for i = 1:numel(c)
        %calculate the centroid of each cell
        [cx,cy] = PolyCentroidNonuniformDensity(v(c{i},1),v(c{i},2), obstacles);
        

        
        cx = min(xrange,max(0, cx));
        cy = min(yrange,max(0, cy));
        if ~isnan(cx) && inpolygon(cx,cy,crs(:,1),crs(:,2))
    
                %Set goals for each agent
                goal_x = -K_prop*(Px(i) - cx);
                goal_y = -K_prop*(Py(i) - cy);
                %check if agents would enter an obstacle as they move to
                %goal
                
             
                obstacle_entered = [];
                for ob =1:size(obstacles,1)
                    for edge =1:size(obstacles,2)
                        vstart = obstacles(ob,edge,1:2);
                        if edge == size(obstacles,2)
                            vend = obstacles(ob,1,1:2);
                        else
                            vend = obstacles(ob,edge+1,1:2);
                        end
                        %Find the intersection point along trajectory of
                        %agent i to its destination
                        [int_x int_y] = polyxpoly([Px(i) goal_x],[Py(i) goal_y],[vstart(1) vend(1)],[vstart(2) vend(2)]);
                        if (~isempty(int_x) || ~isempty(int_y)) 
                            obstacle_entered(size(obstacle_entered,1)+1,1:4) = [ob,edge,int_x,int_y]; 
                        end
                        
                    end
                end
                
                if (isempty(obstacle_entered))
                
                    Px(i) = Px(i) + -K_prop*(Px(i) - cx);
                    Py(i) = Py(i) + -K_prop*(Py(i) - cy);
                    
                else
                    %Find distance along goal path with the minimal intersection point
                    displacement_vec = obstacle_entered(:,3:4) - repmat([Px(i) Py(i)],size(obstacle_entered,1),1);
                    distance_vec = displacement_vec(:,1) .^2 + displacement_vec(:,2) .^ 2;
                    distance_min = (distance_vec == min(distance_vec));
                   
                    min_entry = find(distance_min == 1);
                    
                    %Rescale movement so as to not enter obstacle
                    %Px(i) = obstacle_entered(min_entry,3);
                    %Py(i) = obstacle_entered(min_entry,4);
                    
                    Px(i) = Px(i) + -K_prop*(Px(i) - obstacle_entered(min_entry,3));
                    Py(i) = Py(i) + -K_prop*(Py(i) - obstacle_entered(min_entry,4));
                end
        
        end
    end
    %Store agent locations
	agent_locations(counter,:,1) = Px;
	agent_locations(counter,:,2) = Py;

    if showPlot
        for i = 1:numel(c) % update Voronoi cells
            set(verCellHandle(i), 'XData',v(c{i},1),'YData',v(c{i},2));
        end

        set(titleHandle,'string',['o = Robots, + = Goals, Iteration ', num2str(counter,'%3d')]);
        set(goalHandle,'XData',Px,'YData',Py);%plot goal position
        
        axis equal
        axis([0,xrange,0,yrange]);
        drawnow
%         if mod(counter,50) ==0
%             pause
%             %pause(0.1)
%         end
    end
    
end




function a = triangle_area(P1,P2,P3)
    e12 = sqrt(sum((P1-P2) .^ 2));
    e23 = sqrt(sum((P2-P3) .^ 2));
    e31 = sqrt(sum((P1-P3) .^ 2));
    s = (e12 + e23 + e31)/2;
    a = sqrt(s*(s-e12)*(s-e23)*(s-e31));    


%Simple gaussian density around center point (cx,cy)
function r = density(x,y)
	r = exp(-(x-5)*(x-5)-(y-5)*(y-5));


   
function [Cx,Cy] = PolyCentroidNonuniformDensity(X,Y, obstacles)
    CELL_SAMPLES = 500;
    %Triangulate points - returns a list of indices
    n = size(X,1)-1;
    %Reorient vertices
    x_r = fliplr(X);
    y_r = fliplr(Y);
    
    %Triangulate using script of same name
    triangles = polygon_triangulate(n, x_r(1:n), y_r(1:n));
    
    %Array of areas of each triangle in the polygon
    tri_areas(1:n-2,1) = 0;
    
    %Calculate triangle areas & convert indices to points
    j=1;
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
    DENSITY = zeros(current_sample,1);
    while j < current_sample
        
        %Check to see if random sample is in obstacle
        sample_in_obs_flag = 0;
%         for ob =1:size(obstacles)
%             if inpolygon(random_points(j,1),random_points(j,2),obstacles(ob,:,1), obstacles(ob,:,2))
%                 sample_in_obs_flag = 1;
%                 random_points(j,:) = [0 0];
%                 break;
%             end
%         end
%         if (sample_in_obs_flag == 1)
%             j = j + 1;
%             continue;
%         end
        DENSITY(j,1) = density(random_points(j,1),random_points(j,2));
        numerator_vec_sum = numerator_vec_sum + random_points(j,:)*DENSITY(j,1);
        denominator_sum = denominator_sum + DENSITY(j,1);

        j = j + 1;
    end
   
    

        
    
    C_v = numerator_vec_sum / denominator_sum;
    Cx = C_v(1);
    Cy = C_v(2);

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




