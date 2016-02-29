function agent_locations = approximation_discrete_nonconvex_coverage(numIteration, showPlot, num_agents, obstacles,seed)
close all

rng(seed);

%region size
global xrange;
xrange = 30;  
global yrange;
yrange = 30;

%Setup grid
grid_mat = zeros(xrange,yrange);
density_mat = grid_mat;

agent_locations = zeros(numIteration,num_agents,2);
n = num_agents; %number of robots  (changing the number of robots is interesting)

crs = [ 1, 1;
    1, yrange;
    xrange, yrange;
    xrange, 1];

%Obstruct grid elements who intersect the boundary of the region
% or in obstacles
for i = 1:xrange
    for j = 1:yrange
        %Precompute density for each grid cell.
        density_mat(i,j) = density(i,j);

        if (~inpolygon(i,j, crs(:,1),crs(:,2)))
            grid_mat(i,j) = -1;
        end
        for ob = 1:size(obstacles,1)
            if inpolygon(i,j,obstacles(ob,:,1), obstacles(ob,:,2))
                grid_mat(i,j) = -ob;
            end
        end
    end
end

Px = zeros(n,1);
Py = zeros(n,1);

%Place robots randomly on grid
for i = 1:numel(Px)  
    Px(i) = randi(xrange);
    Py(i) = randi(yrange);

    while (grid_mat(Px(i),Py(i)) ~= 0)
        Px(i) = randi(xrange);
        Py(i) = randi(yrange);
    end

    grid_mat(Px(i),Py(i)) = i;
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
    for i = 1:numel(Px)
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
rng(0,'twister');
NUM_SP_VARS = 2; %occurpied, distance - used in sp-mat

global VISITED;
VISITED = 1;
global DIST;
DIST = 2;

%Create matrix to store shortest path information
sp_mat = zeros(n,xrange,yrange,NUM_SP_VARS); 
%MAtrix  =to assign grid points to agents, inidicating voronoi cell 
voronoi_mat = zeros(xrange,yrange);
cost = zeros(numIteration,1);

% Iteratively Apply LLYOD's Algorithm
for counter = 1:numIteration
    
    if showPlot
            set(currHandle,'XData',Px,'YData',Py);%plot current position
        for i = 1:numel(Px) % color according to
            xD = [get(pathHandle(i),'XData'),Px(i)];
            yD = [get(pathHandle(i),'YData'),Py(i)];
            set(pathHandle(i),'XData',xD,'YData',yD);%plot path position
            set(numHandle(i),'Position',[ Px(i),Py(i)]);
         
        end 

    end
    
       
    %Calculate shortest path to all grid elements
    for i = 1:num_agents     
        sp_mat = dijkstra_sp(i,Px(i),Py(i),sp_mat, grid_mat);
    end
    
    centroid_vec = zeros(n,2);
    density_vec = zeros(n,1);
    cost(counter) = 0;
    
    %Assign each grid cell to one of the voronoi cells
    for x = 1:xrange
        for y = 1:yrange
            if (grid_mat(x,y) < 0)
                voronoi_mat(x,y) = -1;
                continue;
            end
            
            min_dist = Inf;
            min_agent = -1;
            for j =1:num_agents
                
                if sp_mat(j,x,y,DIST) < min_dist
                    min_dist = sp_mat(j,x,y,DIST);
                    min_agent = j;
                end
            end
            assert(min_agent ~= -1);
            voronoi_mat(x,y) = min_agent;

            %calculate the portion of centroid associated with cell
            %one sample of monte carlo integration
            %TODO: find better way to calculate centroid_(x+rand()/10-.05,y+rand()/10 -.05);
            
            centroid_vec(min_agent,1:2) = centroid_vec(min_agent,1:2) + [x y]*density_mat(x,y);

            
            density_vec(min_agent,1) = density_vec(min_agent,1) + density_mat(x,y);
            
        end
    end

    %Get projection of geometric centroids into voronoi cells
    centroid_vec = project_centroid(centroid_vec,density_vec,voronoi_mat);

    %Move each agent to the neighboring cell along the shortest path to the
    %projected centroid (in centroid vec)
    
    for i = 1:num_agents
        current = centroid_vec(i,1:2);
       
        %Backtrace from centroid
        min_dist = Inf;
        prev = current;
        %stop once backtrace reaches the current point
        while (~(current(1) == Px(i) && current(2) == Py(i)))
            
            neighbors = get_neighbors(current,grid_mat);
            
            %Set previous to current so if centroid = agent i location
            %agent will stay still
            prev = current;
            
            %find which neighbor is the closest to the target
            for m = 1:size(neighbors,1)
                v_neighbor = prev + neighbors(m,1:2);
%                 if (v_neighbor(2) == 21)
%                     assert(0);
%                 end
                if (sp_mat(i,v_neighbor(1),v_neighbor(2),DIST) < min_dist)
                    min_dist = sp_mat(i,v_neighbor(1),v_neighbor(2),DIST);
                    current = v_neighbor;
                end
            end
            
        end
        
        %Move agent i
        Px(i) = prev(1); 
        Py(i) = prev(2);
    end
    %Store agent locations
    agent_locations(counter,:,:) = [Px Py];
    if showPlot
        for i = 1:num_agents % update Voronoi cells
            %TODO - find a way to plot cells
            X = [];
            Y = [];
            for x = 1:xrange
                for y=1:yrange
                    if voronoi_mat(x,y) == i
                        X = [X;x];
                        Y = [Y;y];
                    end
                end
            end
           
            set(verCellHandle(i), 'XData',X,'YData',Y);
            
        end

        set(titleHandle,'string',['o = Robots, + = Goals, Iteration ', num2str(counter,'%3d')]);
        set(goalHandle,'XData',Px,'YData',Py);%plot goal position
        
        for x =1:xrange
            for y=1:yrange


                %plot voronoi cells
                %plot centroid
            end
        end

        
        
        axis equal
        axis([0,xrange,0,yrange]);
        drawnow
    end
    
end


%Return a relative list of movements from 'point'
function neighbors = get_neighbors(point,grid_mat)
    global xrange;
    global yrange;
    
    neighbors = [];
    %if not an obstacle
    if (point(1) < xrange && grid_mat(point(1)+1,point(2)) >= 0)
        neighbors = [neighbors;1 0];
    end
    if (point(1) < xrange && point(2) < yrange && grid_mat(point(1)+1,point(2)+1) >= 0)
         neighbors = [neighbors;1 1];
    end
    if (point(2) < yrange && grid_mat(point(1),point(2)+1) >= 0)
     neighbors = [neighbors;0 1];    
    end
    if (point(1) > 1 && point(2) < yrange && grid_mat(point(1)-1,point(2)+1) >= 0)
         neighbors = [neighbors;-1 1];
    end
    if (point(1) > 1 && grid_mat(point(1)-1,point(2)) >= 0)
         neighbors = [neighbors;-1 0];
    end
    if (point(1) > 1 && point(2) > 1 && grid_mat(point(1)-1,point(2)-1) >= 0)
         neighbors = [neighbors;-1 -1];
    end
    if (point(2) > 1 && grid_mat(point(1),point(2)-1) >= 0)
         neighbors = [neighbors;0 -1];
    end
    if (point(1) < xrange && point(2) > 1 && grid_mat(point(1)+1,point(2)-1) >= 0)
         neighbors = [neighbors;1 -1];
    end


function d = euc_dist(p1,p2)
    d = sqrt(sum((p1 - p2) .^ 2));

%Return projected centroids, as defined in Battacharya, et al 
function centroid_vec = project_centroid(centroid_vec,density_vec,voronoi_mat)
    global xrange;
    global yrange;

    num_agents = size(centroid_vec,1);
    
    %Calculate centroids
    for i = 1:num_agents
       
        centroid_vec(i,1:2) = centroid_vec(i,1:2) ./ density_vec(i);
        
    end
    
    min_dist_vec(1:num_agents,1:3) = Inf;
    
    for x=1:xrange
        for y=1:yrange
            
            agent_num = voronoi_mat(x,y);
            %if an agent is linked to a point in grid (i,e voronoi cell is
            %not in an obstacle - value > 0), and distance to centroid is
            %is less than minimum observed distance
            if (agent_num > 0 && (min_dist_vec(agent_num,1) > euc_dist([x y], centroid_vec(agent_num,1:2))))
                min_dist_vec(agent_num,1) = euc_dist([x y], centroid_vec(agent_num,1:2));
                min_dist_vec(agent_num,2) = x;
                min_dist_vec(agent_num,3) = y;
                
            end
        end
    end
    
    centroid_vec(:,1:2) = min_dist_vec(:,2:3);
    

%sp_mat is a (xrange,yrange) matrix of cells
function sp_mat = dijkstra_sp(agent,startx,starty,sp_mat,grid_mat)
global VISITED;
global DIST;
global xrange;
global yrange;

%Reset all sp_mat elements to unvisited
sp_mat(agent,:,:,VISITED) = 0;
sp_mat(agent,:,:,DIST) = Inf;
    


%VISITED 2 equiv. to current node
sp_mat(agent,startx,starty,VISITED) = 2;
sp_mat(agent,startx,starty,DIST) = 0;


%current x and y position
cx = startx;
cy = starty;

mx = -1;
my = -1;
exit_flag = 0;

while (exit_flag == 0)
    %Assign tentative weights to 8 neighbors of cx,cy
    
        smallest_tent_dist = Inf;
        %N
        if (cy < yrange && grid_mat(cx,cy+1) >= 0)
            if (sp_mat(agent,cx,cy+1,DIST) > sp_mat(agent,cx,cy,DIST) + 1)
                sp_mat(agent,cx,cy+1,DIST) = sp_mat(agent,cx,cy,DIST) + 1;

                %update move-to node
                if (sp_mat(agent,cx,cy+1,DIST) < smallest_tent_dist)
                    smallest_tent_dist = sp_mat(agent,cx,cy+1,DIST);
                    mx = cx;
                    my = cy +1;
                end
            end
        end

        %NE
        if (cy < yrange && cx < xrange && grid_mat(cx+1,cy+1) >= 0)
            if (sp_mat(agent,cx+1,cy+1,DIST) > sp_mat(agent,cx,cy,DIST) + sqrt(2))
                sp_mat(agent,cx+1,cy+1,DIST) = sp_mat(agent,cx,cy,DIST) + sqrt(2);
                %update move-to node
                if (sp_mat(agent,cx+1,cy+1,DIST) < smallest_tent_dist)
                    smallest_tent_dist = sp_mat(agent,cx+1,cy+1,DIST);
                    mx = cx+1;
                    my = cy +1;
                end
            end
        end
        %E
        if (cx < xrange && grid_mat(cx+1,cy) >= 0)
            if (sp_mat(agent,cx+1,cy,DIST) > sp_mat(agent,cx,cy,DIST) + 1)
                sp_mat(agent,cx+1,cy,DIST) = sp_mat(agent,cx,cy,DIST) + 1;
                            %update move-to node
                if (sp_mat(agent,cx+1,cy,DIST) < smallest_tent_dist)
                    smallest_tent_dist = sp_mat(agent,cx+1,cy,DIST);
                    mx = cx + 1;
                    my = cy;
                end
            end
        end
        %SE
        if (cy > 1 && cx < xrange && grid_mat(cx+1,cy-1) >= 0)
            if (sp_mat(agent,cx+1,cy-1,DIST) > sp_mat(agent,cx,cy,DIST) + sqrt(2))
                sp_mat(agent,cx+1,cy-1,DIST) = sp_mat(agent,cx,cy,DIST) + sqrt(2);

                if (sp_mat(agent,cx+1,cy-1,DIST) < smallest_tent_dist)
                    smallest_tent_dist = sp_mat(agent,cx+1,cy-1,DIST);
                    mx = cx + 1;
                    my = cy - 1;
                end
            end
        end
        %S
        if (cy > 1 && grid_mat(cx,cy-1) >= 0)
            if (sp_mat(agent,cx,cy-1,DIST) > sp_mat(agent,cx,cy,DIST) + 1)
                sp_mat(agent,cx,cy-1,DIST) = sp_mat(agent,cx,cy,DIST) + 1;

                if (sp_mat(agent,cx,cy-1,DIST) < smallest_tent_dist)
                    smallest_tent_dist = sp_mat(agent,cx,cy-1,DIST);
                    mx = cx ;
                    my = cy - 1;
                end
            end
        end
        %SW
            if (cy > 1 && cx > 1 && grid_mat(cx-1,cy-1) >= 0)
            if (sp_mat(agent,cx-1,cy-1,DIST) > sp_mat(agent,cx,cy,DIST) + sqrt(2))
                sp_mat(agent,cx-1,cy-1,DIST) = sp_mat(agent,cx,cy,DIST) + sqrt(2);

                if (sp_mat(agent,cx-1,cy-1,DIST) < smallest_tent_dist)
                    smallest_tent_dist = sp_mat(agent,cx-1,cy-1,DIST);
                    mx = cx - 1;
                    my = cy - 1;
                end
            end
        end
        %W
        if (cx > 1 && grid_mat(cx-1,cy) >= 0)
            if (sp_mat(agent,cx-1,cy,DIST) > sp_mat(agent,cx,cy,DIST) + 1)
                sp_mat(agent,cx-1,cy,DIST) = sp_mat(agent,cx,cy,DIST) + 1;

                if (sp_mat(agent,cx-1,cy,DIST) < smallest_tent_dist)
                    smallest_tent_dist = sp_mat(agent,cx-1,cy,DIST);
                    mx = cx - 1;
                    my = cy ;
                end
            end
        end
        %NW
        if (cy < yrange && cx > 1 && grid_mat(cx-1,cy+1) >= 0)
            if (sp_mat(agent,cx-1,cy+1,DIST) > sp_mat(agent,cx,cy,DIST) + sqrt(2))
                sp_mat(agent,cx-1,cy+1,DIST) = sp_mat(agent,cx,cy,DIST) + sqrt(2);

             if (sp_mat(agent,cx-1,cy+1,DIST) < smallest_tent_dist)
                    smallest_tent_dist = sp_mat(agent,cx-1,cy+1,DIST);
                    mx = cx - 1;
                    my = cy + 1;
                end
            end
        end

        sp_mat(agent,cx,cy,VISITED) = 1;

    %ifind smallest distance amongst all unvisited nodes
    %HACK - change for efficiency, use distances calculated above
    min_dist = Inf;
    mx = -1;
    my = -1;
    for x =1:xrange
        for y = 1:yrange
            %not an obstruction, unvisited and min_
            if (grid_mat(x,y) >= 0 && sp_mat(agent,x,y,VISITED) == 0 && sp_mat(agent,x,y,DIST) < min_dist)
                mx = x;
                my = y;
                min_dist = sp_mat(agent,x,y,DIST);
            end
        end
    end


    %DEBUG
    %if no locations can be moved to, check all locations have been visited
    if (mx == -1 && my == -1)
        for x =1:xrange
            for y = 1:yrange
                if (sp_mat(agent,x,y,VISITED) == 0 && grid_mat(x,y) >= 0 )
                    assert(0);
                end
            end
        end
        exit_flag = 1;
        break;
    end
    
    %Update current node to mx, my
    sp_mat(agent,mx,my,VISITED) = 2;
    
    cx = mx;
    cy = my;
end %end while
    


%A toy density function - Gaussian density around center point (cx,cy)
function r = density(x,y)
	r = exp(-(x-5)*(x-5)-(y-5)*(y-5));
