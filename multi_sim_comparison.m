%obstacle configuration - numerical id for obstacle configurations described in get_obstacle_set
% density_function_type - UNIMPLEMENTED
%density_function_params - UNIMPLEMENTED
%num_agents - as described, integer input
%num_iterations - number of iterations to compute for each simulation
function agent_loc = multi_sim_comparison(obstacle_configuration, density_function_type,density_function_params, num_agents,num_iterations, agent_loc)

%random seed
seed = 16;

%Setup obstacles
obstacles = get_obstacle_set(obstacle_configuration);

%For now show plots of algorithm runs
show_plot = true;

%If agent_loc is not passed as argument
if nargin < 6
    
    %Run the 4 algorithms using params specified in function input
    agent_loc = zeros(4,num_iterations,num_agents,2);

    %Set control gain
    control_gain = 0.1;

    %Run lloyd style algorithm
    agent_loc(1,:,:,:) = loydsAlgorithm_nonuniform(num_iterations,show_plot,num_agents,obstacles,seed,control_gain);

    %Run approximation via search based grid algorithm
    agent_loc(2,:,:,:) = approximation_discrete_nonconvex_coverage(num_iterations,show_plot,num_agents,obstacles,seed);

    %Run combined tangentbug and lloyd
    loop_gain = 3;
    max_step = 0.25;
    %agent_loc(3,:,:,:) = combined(num_iterations,show_plot,num_agents,obstacles,seed,control_gain,loop_gain,max_step);

    %Run optimal annealing, algorithm
    %agent_loc(4,:,:,:) = optimal_coverage_grid(num_iterations,show_plot,num_agents,obstacles,seed);
end

%Plot cost functions

v = 1:num_iterations;
NUM_SAMPLES = 1000;
cost_lloyd  = get_cost_timeline(agent_loc(1,:,:,:),obstacles,NUM_SAMPLES);
cost_approx  = get_cost_timeline(agent_loc(2,:,:,:),obstacles,NUM_SAMPLES);
cost_combined = get_cost_timeline(agent_loc(3,:,:,:),obstacles,NUM_SAMPLES);
cost_optimal  = get_cost_timeline(agent_loc(4,:,:,:),obstacles,NUM_SAMPLES);


figure(2);
plot(v,cost_lloyd,'o',v,cost_approx,'+',v, cost_combined,'x',v,cost_optimal,'^');
title('Coverage Control Sampled (1000) Cost');
xlabel('Iterations');
ylabel('Cost');


NUM_SAMPLES = 3000;

cost_lloyd = get_cost_timeline(agent_loc(1,:,:,:),obstacles,NUM_SAMPLES);
cost_approx = get_cost_timeline(agent_loc(2,:,:,:),obstacles,NUM_SAMPLES);
cost_combined = get_cost_timeline(agent_loc(3,:,:,:),obstacles,NUM_SAMPLES);
cost_optimal  = get_cost_timeline(agent_loc(4,:,:,:),obstacles,NUM_SAMPLES);


figure(3);
plot(v,cost_lloyd,'o',v,cost_approx,'+',v, cost_combined,'x',v,cost_optimal,'^');
title('Coverage Control Sampled (3000) Cost');
xlabel('Iterations');
ylabel('Cost');

end

%Add an elseif clause to add new obstacles
%Format of obstacles are M x N x N, whre M is number of obstacles, N is
%vertices of obstacles.
%We assume boundary is 30x30
function obstacles = get_obstacle_set(ob_config)
    obstacles = [];
    if (ob_config == 1)
        obstacles = [];
    end
    if mod(ob_config,2) == 0
        %one saw shape osbtacle in bottom left corner
        obstacles(size(obstacles,1)+1,:,:) = [0,0;10,0;10,10;2,7;7,4;0,0];
    end
    if mod(ob_config,3) == 0
        % 
          obstacles(size(obstacles,1)+1,:,:) = [0,0;20,0;20,10;3,10;3,6;7,6;7,4;3,4;0,0];
    end
    if mod(ob_config,5) == 0
        obstacles(size(obstacles,1)+1,:,:) = [5,25;25,5;15,10;5,25;5,25;5,25;5,25;5,25;5,25];
    end
    
end

%Use sampling to detesrmine global cost
function cost_vec = get_cost_timeline(agent_locations,obstacles, NUM_SAMPLES)
	xrange = 30;
	yrange = 30;
    
    if min(agent_locations == 0) == 1
        cost_vec = zeros(size(agent_locations,2));
        return;
    end
        
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
    
    cost_vec = cost_vec ./ NUM_SAMPLES;

end

function r = density(x,y)
	r = exp(-(x-5)*(x-5)-(y-5)*(y-5));
end
