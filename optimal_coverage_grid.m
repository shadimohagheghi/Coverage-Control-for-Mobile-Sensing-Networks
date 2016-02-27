function [Px, Py,COST] = optimal_coverage_grid(Px,Py, crs, numIterations, showPlot)

close all

seed = 2;
rng(seed);

num_agents = 5;

if nargin < 1   % demo mode
    showPlot = true;
    numIterations  = 1000;
    xrange = 30;  %region size
    yrange = 30;
    n = num_agents; %number of robots  (changing the number of robots is interesting)

    
    crs = [ 0, 0;
        0, yrange;
        xrange, yrange;
        xrange, 0];
    
    %Build grid array - Assume x, y start at 0
    grid = zeros(floor(xrange)+1,floor(yrange)+1);
    
    obstacles(1,:,:) = [3,2;10,2;10,20;3,20;3,8;6,8;6,6;3,6];
        
    %Obstruct grid elements who intersect the boundary of the region
    for i = 0:size(grid,1)-1
        for j = 0:size(grid,2)-1
            if (~inpolygon(i,j, crs(:,1),crs(:,2)))
                grid(i+1,j+1) = -1;
            end
            
            for ob = 1:size(obstacles,1)
                if inpolygon(i,j,obstacles(ob,:,1), obstacles(ob,:,2))
                    grid(i,j) = -ob;
                end
            end
        end
    end
    
    Px = zeros(n,1);
    Py = zeros(n,1);
    
    %Place robots randomly on grid
    for i = 1:numel(Px)  
        Px(i) = randi(size(grid,1));
        Py(i) = randi(size(grid,2));

        while (grid(Px(i)+1,Py(i)+1) ~= 0)
            Px(i) = randi(size(grid,1));
            Py(i) = randi(size(grid,2));
        end
        
        grid(Px(i)+1,Py(i)+1) = i;
    end
else
    xrange = max(crs(:,1));
    yrange = max(crs(:,2));
    n = numel(Px); %number of robots  
end

%%%%%%%%%%%%%%%%%%%%%%%% VISUALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if showPlot
    verCellHandle = zeros(n,1);
    cellColors = hsv(n+size(obstacles,1));
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
COST = zeros(numIterations*n,1);

% Iteratively Apply Optimal Coverage Algorithm by Hou et al.
for counter = 1:numIterations
            %DEBUG
        if counter == 9
            assert(true);
        end
    if showPlot
        set(currHandle,'XData',Px,'YData',Py);%plot current position
        for i = 1:numel(Px) % color according to
            xD = [get(pathHandle(i),'XData'),Px(i)];
            yD = [get(pathHandle(i),'YData'),Py(i)];
            set(pathHandle(i),'XData',xD,'YData',yD);%plot path position
            set(numHandle(i),'Position',[ Px(i),Py(i)]);
        end 
    end
               
   %Px,PY are grid location vectors
    for i = 1:n
        [v,c] = VoronoiBounded(Px, Py,crs);
        
        %current_cost = calculateCoverageCost(grid,Px,Py,v,c);
        current_cost = calculateContinousCoverageCost(Px,Py,v,c);
        
        COST(((counter-1)*n)+i) = current_cost;
        
        %Compute voronoi partitions
        cost_diff = [0; 0; 0; 0];
        
        %For each of the up to four directions, calculte the voronoi cell
        %and coverage cost
        neighbors_free = [-1];
        %North
        if (Py(i) + 1 <= floor(yrange) && grid(Px(i)+1,Py(i) + 2) == 0)
            Py(i) = Py(i) + 1;
            [v,c] = VoronoiBounded(Px+rand(n,1)/100, Py+rand(n,1)/100,crs);
            %cost_diff(1,1) = calculateCoverageCost(grid,Px,Py,v,c) - current_cost;
            cost_diff(1,1) = calculateContinousCoverageCost(Px,Py,v,c) - current_cost;
            Py(i) = Py(i) - 1;
            
            neighbors_free(size(neighbors_free,2)+1) = 1;
        end
        %South
        if (Py(i) - 1 >= 0 && grid(Px(i)+1,Py(i) ) == 0)
            Py(i) = Py(i) - 1;
            [v,c] = VoronoiBounded(Px+rand(n,1)/100, Py+rand(n,1)/100,crs);
            %cost_diff(2,1) = calculateCoverageCost(grid,Px,Py,v,c) - current_cost;
            cost_diff(2,1) = calculateContinousCoverageCost(Px,Py,v,c) - current_cost;
            Py(i) = Py(i) + 1;
            neighbors_free(size(neighbors_free,2)+1) = 2;
        end
        %East
        if (Px(i) + 1 <= floor(xrange) && grid(Px(i) + 2,Py(i)+1)== 0)
            Px(i) = Px(i) + 1;
            [v,c] = VoronoiBounded(Px+rand(n,1)/100, Py+rand(n,1)/100,crs);
            %cost_diff(3,1) = calculateCoverageCost(grid,Px,Py,v,c) - current_cost;
            cost_diff(3,1) = calculateContinousCoverageCost(Px,Py,v,c) - current_cost;
            Px(i) = Px(i) - 1;
            
            neighbors_free(size(neighbors_free,2)+1) = 3;
        end
        %West
        if (Px(i) - 1 >= 0 && grid(Px(i) ,Py(i)+1) == 0)
            Px(i) = Px(i)  - 1;
            [v,c] = VoronoiBounded(Px+rand()/100, Py+rand()/100,crs);
            %cost_diff(4,1) = calculateCoverageCost(grid,Px,Py,v,c) - current_cost;
            cost_diff(4,1) = calculateContinousCoverageCost(Px,Py,v,c) - current_cost;
            Px(i) = Px(i) + 1;
            
            neighbors_free(size(neighbors_free,2)+1) = 4;
        end
        
        %remove -1 from front of neighbors free array
        neighbors_free = neighbors_free(2:size(neighbors_free,2));
        
        for iter = 1:size(neighbors_free,2)
            if (cost_diff(iter,1) < 0)
                cost_diff(iter,1) = 0;
            end
        end
        move_probabilities = zeros(size(neighbors_free,2),1);
        if size(neighbors_free,2) >= 1
            
            max_cost = max(cost_diff);
            %Use probabilities taken from exp(-cost_diff/alpha(t)) to generate a random move
            move_probabilities = exp(-cost_diff/alpha(counter,n,size(grid,1)*size(grid,2),max_cost))...
                        * (1/(size(neighbors_free,2)-1));
        
        
            move_interval_prob = zeros(size(neighbors_free,2),1);
            for iter = 1:size(neighbors_free,2)
                if iter > 1
                    move_interval_prob(iter,1) = move_interval_prob(iter-1,1) + move_probabilities(neighbors_free(1,iter),1);
                else
                    move_interval_prob(iter,1) = move_probabilities(neighbors_free(1,iter),1);
                end
            end

                  %Generate the move randomly
            r = rand();
            move = -1;
            for iter = 1:(size(neighbors_free,2))
                if r < move_interval_prob(iter,1)
                    move = neighbors_free(1,iter);
                    break;
                end
            end
            
            %Move point and update grid location
            %Assert no other point is in the moved-to-location
            if move == 1 %North
                assert(grid(Px(i)+1,Py(i)+1) == i);
                grid(Px(i)+1,Py(i)+1) = 0;
                Py(i) = Py(i) + 1;
                assert(grid(Px(i)+1,Py(i)+1) == 0);
                grid(Px(i)+1,Py(i)+1) = i;
            elseif move == 2 %South
                assert(grid(Px(i)+1,Py(i)+1) == i);
                grid(Px(i)+1,Py(i)+1) = 0;
                Py(i) = Py(i) - 1;
                assert(grid(Px(i)+1,Py(i)+1) == 0);
                grid(Px(i)+1,Py(i)+1) = i;
            elseif move == 3 %East
                assert(grid(Px(i)+1,Py(i)+1) == i);
                grid(Px(i)+1,Py(i)+1) = 0;
                Px(i) = Px(i) + 1;
                assert(grid(Px(i)+1,Py(i)+1) == 0);
                grid(Px(i)+1,Py(i)+1) = i;
            elseif move == 4 %West
                assert(grid(Px(i)+1,Py(i)+1) == i);
                grid(Px(i)+1,Py(i)+1) = 0;
                Px(i) = Px(i) - 1;
                assert(grid(Px(i)+1,Py(i)+1) == 0);
                grid(Px(i)+1,Py(i)+1) = i;
            end
        end
        
    end
     
    
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
%Plot cost function
V = 1:numIterations*n;
figure (2);
plot(V,(COST));

%TODO: pass in values of r and m
%TODO: do a real calculation of max_cost
function a = alpha(counter,m,grid_area,max_cost)

%a = m*grid_area*max_cost/log(counter+1);
a = max_cost/log(counter+1);


function cost = calculateContinousCoverageCost(Px,Py,v,c)
cost = 0;
for i = 1:numel(c)
    cost_i = CellCost(Px(i),Py(i),v(c{i},1),v(c{i},2));
     cost = cost + cost_i;
end
    
    
% Calculate coverage cost for robots in the grid
function cost = calculateCoverageCost(grid,Px,Py, v,c)
    
cost = 0;
%x and y are locations
%grid(x+1,y+1) are the matrix entries 
    for x = 0:size(grid,1)-1
        for y = 0:size(grid,2)-1
            if (grid(x+1,y+1) >= 0) %i.e. not inside the boundary/in an obstacle
                for iter = 1:size(Px,2) %Check if point in voronoi cell 'iter'
                   if (inpolygon(x,y,v(c{iter},1),v(c{iter},2)))
                       
                       cost = cost + ...
                           performance(Px(iter),Py(iter),x,y) * ...
                           density(x,y);
                       break;
                   end %if
                end %for
            end%if
        end
    end
 
function [cost] = CellCost(px,py,X,Y)
    CELL_SAMPLES = 50;
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
    DENSITY = zeros(current_sample,1);
    while j < current_sample
        DENSITY(j,1) = density(random_points(j,1),random_points(j,2));
        j = j + 1;
    end
   


    %Calculate cell cost using previously calculated centroid
    j = 1;
    cost = 0;
    while j < current_sample
        %Use norm square of point locations:
        cost = cost + ((random_points(j,1) - px) ^ 2 + (random_points(j,2) - py) ^ 2 ) * DENSITY(j,1);
        j = j + 1;
    end
    
    %Monte carlo integrated approximation of cost function from Cortes et
    %al 2004.
    cost = total_area * cost / current_sample;

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

function a = triangle_area(P1,P2,P3)
    e12 = sqrt(sum((P1-P2) .^ 2));
    e23 = sqrt(sum((P2-P3) .^ 2));
    e31 = sqrt(sum((P1-P3) .^ 2));
    s = (e12 + e23 + e31)/2;
    a = sqrt(s*(s-e12)*(s-e23)*(s-e31));    

function f_x = performance(cx,cy,px,py)
f_x = sqrt((cx-px) ^2 + (cy-py)^2);

%Density function - Gaussian density around center points 
function r = density(x,y)
   %r = exp(-((x-1)*(x-1)) + -((y-5)*(y-5)));
   r = exp( (-(x-5)*(x-5) + -(y-5)*(y-5)));

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




