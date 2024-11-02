% run_best_pid.m

%% Load Optimized PID Parameters
load('./results/optimized_pid_results.mat', 'results');

% Extract the optimized PID parameters (Kp, Ki, Kd)
Kp = results.best_pid(1);
Ki = results.best_pid(2);
Kd = results.best_pid(3);

fprintf('Using Optimized PID Parameters:\n');
fprintf('Kp = %.4f, Ki = %.4f, Kd = %.4f\n', Kp, Ki, Kd);

%% Set PID Parameters in Simulink Model
model = 'simulation_test';
load_system(model);

% Set the PID parameters in the model
set_param([model '/PID Controller'], 'P', num2str(Kp));
set_param([model '/PID Controller'], 'I', num2str(Ki));
set_param([model '/PID Controller'], 'D', num2str(Kd));

%% Run Simulation
% Run the model with the optimized parameters
sim(model, 'SimulationMode', 'normal', 'StopTime', '100');
