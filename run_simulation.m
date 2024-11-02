clc, close all, clear workspace
%% Particle Swarm Optimization (PSO) for PID Controller Tuning
% This script implements the PSO algorithm to optimize the PID controller parameters.
% It minimizes a fitness function that combines the error and overshoot of the system.

%% Initialization
% Define the parameters for the PSO algorithm and initialize particle positions and velocities.

n = 50;           % Number of particles in the swarm
max_steps = 10;   % Maximum number of iterations (generations)
dim = 3;          % Dimensionality of the problem (Kp, Ki, and Kd for PID)

% PSO acceleration coefficients
c1 = 1.2;         % Cognitive coefficient (personal best)
c2 = 0.12;        % Social coefficient (global best)
w = 0.9;          % Inertia weight to control the velocity

% Initialize particle positions and velocities
% Positions are randomly initialized within the range [-5, 5] for each dimension
position = 10 * (rand(dim, n) - 0.5);  

% Velocities are initialized with small random values from a normal distribution
velocity = 0.3 * randn(dim, n);        

% Initialize local (personal) and global best positions and fitness values
local_best_position = position;          % Personal best positions start as initial positions
local_best_fitness = inf(n, 1);         % Initialize personal best fitness to infinity
global_best_fitness = inf;               % Initialize global best fitness to infinity
global_best_position = zeros(dim, 1);    % Initialize global best position

% Initialize random factors R1 and R2 for velocity updates
R1 = rand(dim, n);  % Random values for cognitive component
R2 = rand(dim, n);  % Random values for social component

% Initialize arrays to store best fitness and best PID parameters per iteration
best_fitness_history = zeros(max_steps, 1);          % To store best fitness at each iteration
best_pid_history = zeros(dim, max_steps);            % To store best PID parameters at each iteration

%% Main Loop for PSO
% Iterate through each generation to update particle positions and velocities
for iter = 1:max_steps
    % Evaluate fitness of each particle in the swarm
    for i = 1:n
        % Compute the fitness using the fitness_function for the current position
        fitness = fitness_function(position(:, i));
        
        % Update personal best if the current fitness is better
        if fitness < local_best_fitness(i)
            local_best_fitness(i) = fitness;
            local_best_position(:, i) = position(:, i);
        end
    end

    % Determine the current global best fitness and its corresponding particle
    [current_global_best_fitness, best_idx] = min(local_best_fitness);
    
    % Update global best if a better fitness is found
    if current_global_best_fitness < global_best_fitness
        global_best_fitness = current_global_best_fitness;
        global_best_position = local_best_position(:, best_idx);
    end

    % Store the best fitness and PID parameters for this iteration
    best_fitness_history(iter) = global_best_fitness;
    best_pid_history(:, iter) = global_best_position;

    % Update random factors for velocity update to introduce stochasticity
    R1 = rand(dim, n);  % New random values for cognitive component
    R2 = rand(dim, n);  % New random values for social component

    % Update velocities based on inertia, cognitive, and social components
    velocity = w * velocity ...                                     % Inertia component
             + c1 * R1 .* (local_best_position - position) ...    % Cognitive component
             + c2 * R2 .* (global_best_position - position);      % Social component

    % Update particle positions based on the new velocities
    position = position + velocity;

    % Display the progress of the current iteration
    fprintf('Iteration %d, Best Fitness: %.6f\n', iter, global_best_fitness);
end

%% Display Results
% After completing all iterations, display the best position and fitness found.

fprintf('Optimization finished after %d iterations.\n', max_steps);
fprintf('Best position found: [%f, %f, %f]\n', ...
    global_best_position(1), global_best_position(2), global_best_position(3));
fprintf('Best fitness found: %.6f\n', global_best_fitness);

%% Plotting Results

% Plot Best Fitness Over Iterations
figure;
plot(1:max_steps, best_fitness_history, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Best Fitness');
title('Best Fitness Over Iterations');
grid on;

% Plot PID Parameter Convergence (Optional)
figure;
hold on;
colors = ['r', 'g', 'b'];  % Colors for Kp, Ki, Kd
labels = {'Kp', 'Ki', 'Kd'};
for d = 1:dim
    plot(1:max_steps, best_pid_history(d, :), 'Color', colors(d), 'LineWidth', 2, 'DisplayName', labels{d});
end
xlabel('Iteration');
ylabel('PID Parameter Value');
title('PID Parameter Convergence');
legend('show');
grid on;
hold off;

%% Save Results

% Create a structure to store the results
results = struct();
results.best_pid = global_best_position;          % Optimized PID parameters [Kp; Ki; Kd]
results.best_fitness = global_best_fitness;       % Best fitness value
results.best_fitness_history = best_fitness_history;  % History of best fitness over iterations
results.best_pid_history = best_pid_history;          % History of best PID parameters over iterations

% Define the filename (you can customize the path and name as needed)
filename = './results/optimized_pid_results.mat';

% Save the structure to a .mat file
save(filename, 'results');

fprintf('Optimized PID parameters and results have been saved to %s\n', filename);

%% Fitness Function: fitness_function
% This function evaluates the performance of the PID controller based on the
% current PID parameters. It computes an objective function that combines error and overshoot.

function F = fitness_function(pid)
    % fitness_function - Evaluates the fitness of PID parameters
    %
    % Syntax: F = fitness_function(pid)
    %
    % Inputs:
    %   pid - A 3-element array containing [Kp, Ki, Kd] for the PID controller
    %
    % Outputs:
    %   F   - The computed fitness value (objective function)

    % Extract PID parameters
    Kp = pid(1);   % Proportional gain
    Ki = pid(2);   % Integral gain
    Kd = pid(3);   % Derivative gain

    % Display current PID parameters for debugging
    fprintf('Evaluating PID parameters: Kp = %.2f, Ki = %.2f, Kd = %.2f\n', Kp, Ki, Kd);

    % Set up simulation options for the Simulink model 'simulation'
    sim_options = simset('solver', 'ode5', 'SrcWorkspace', 'Current', 'DstWorkspace', 'Current');

    % Run the Simulink simulation from time 0 to 100 seconds
    [~, ~, yout] = sim('simulation_pid', [0 100], sim_options);

    % Calculate the error signal as the difference between the output and desired value (1)
    error_signal = yout - 1;

    % Calculate the maximum overshoot from the simulation output
    sys_overshoot = max(yout) - 1;

    % Define weighting factors for error and overshoot in the objective function
    alpha = 10;  % Weight for overshoot
    beta = 10;   % Weight for error

    % Compute the fitness value as a weighted sum of squared error and overshoot
    F = sum(error_signal.^2) * beta + sys_overshoot * alpha;
end
