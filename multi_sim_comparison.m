function multi_sim_comparison(obstacle_configuration, density_function_type,density_function_params, num_agents,num_iterations)

%random seed
seed = 10;

%Setup obstacles
obstacles = get_obstacle_set(obstacle_configuration)

%For now show plots of algorithm runs
show_plot = true;

%Run the 4 algorithms using params specified in function input
%Set control gaintrue
control_gain = 0.1;
agent_loc(1,:,:,:) = loydsAlgorithm_nonuniform(num_iterations,show_plot,num_agents,obstacles,seed,control_gain);
agent_loc(2,:,:,:) = approximation_discrete_nonconvex_coverage(num_iterations,show_plot,num_agents,obstacles,seed);
%agent_loc(3,:,:,:) = combined
%agent_loc(4,:,:,:) = optimal_coverage_grid

%Plot cost functions

v = 1:num_iterations;
cost = get_cost_timeline(agent_loc(1,:,:,:),obstacles);
figure(2);
plot(v,cost);

cost = get_cost_timeline(agent_loc(2,:,:,:),obstacles);
figure(3);
plot(v,cost);


end

%Add an elseif clause to add new obstacles
%Format of obstacles are M x N x N, whre M is number of obstacles, N is
%vertices of obstacles.
%We assume boundary is 30x30
function obstacles = get_obstacle_set(ob_config)
    obstacles = [];
    if (ob_config == 1)
        obstacles = [];
    elseif ob_config == 2
        obstacles(1,:,:) = [0,0;10,0;10,10;2,7;7,4;0,0];
        %obstacles(2,:,:) = [
        %obstacles(3,:,:) = [
        %obstacles(4,:,:) = [
            
    end

end

%Use sampling to detesrmine global cost
function cost_vec = get_cost_timeline(agent_locations,obstacles)
	NUM_SAMPLES = 2000;
	xrange = 30;
	yrange = 30;
	sample_points(:,1) = rand(NUM_SAMPLES,1)*xrange;
	sample_points(:,2) = rand(NUM_SAMPLES,1)*yrange;
	sample_points(:,3) = zeros(NUM_SAMPLES,1);
	for i =1:NUM_SAMPLES
		for ob = 1:size(obstacles,1)

			if inpolygon(sample_points(i,1), sample_points(i,2),obstacles(ob,:,1),obstacles(ob,:,2))
				sample_points(i,3) = -1;
				break;
			end
		end
	end

	cost_vec = zeros(size(agent_locations,2),1);
	for counter =1:size(agent_locations,2)
        for i=1:NUM_SAMPLES
            if (sample_points(i,3) == 0)
            min_dist = Inf;
            min_agent = -1;
                for ag = 1:size(agent_locations,3)
                    dist = sqrt(sum( ([agent_locations(1,counter,ag,1) agent_locations(1,counter,ag,2)] - sample_points(i,1:2)) .^ 2));% * density(sample_points(i,1:2));
                    if dist < min_dist
                        min_agent = ag;
                        min_dist = dist;
                    end
                end
                assert(min_agent ~= -1);
                
                cost_vec(counter,1) = cost_vec(counter,1) + min_dist * density(sample_points(i,1),sample_points(i,2));
            end
        end
	end

end

function r = density(x,y)
	r = exp(-(x-5)*(x-5)-(y-5)*(y-5));
end
