# Particle Swarm Optimization for PID Controller Tuning

This project demonstrates the use of Particle Swarm Optimization (PSO) to fine-tune the parameters of a PID (Proportional-Integral-Derivative) controller. By minimizing a fitness function that evaluates error and overshoot, PSO optimizes the PID gains to achieve optimal control performance. The project is implemented in MATLAB, utilizing a custom fitness function and Simulink simulations to assess each PID configuration.

## Project Summary

In this project, a PSO algorithm iteratively adjusts the values of the PID controller parameters—Kp, Ki, and Kd—based on their effectiveness at minimizing control error and reducing overshoot. The algorithm is designed to balance exploration and exploitation, where particles move toward both their personal best solutions and the global best found by the swarm. This process is repeated over multiple iterations, converging on an optimal set of PID parameters that yield improved performance in controlling the target system.

### Key Concepts and Approach

1. **PSO Algorithm**: A computational method inspired by the social behavior of swarms, PSO is used to find the optimal PID parameters. Particles, or candidate solutions, explore the search space and adjust their positions based on personal and global best values. The algorithm uses a balance of cognitive and social components to guide each particle.

2. **PID Controller Tuning**: The controller parameters (Kp, Ki, Kd) directly influence the response of a dynamic system. This project aims to find values that minimize a custom-defined fitness function representing the performance of the PID control.

3. **Fitness Function**: The fitness function evaluates how well each particle's PID configuration performs. This is measured based on two main criteria:
   - **Error**: The cumulative squared error between the system's output and the desired setpoint.
   - **Overshoot**: The degree to which the system output exceeds the target.

4. **Simulink Simulation**: For each set of PID parameters, the system response is simulated in Simulink to calculate the error and overshoot, feeding into the fitness function.

### Project Structure

- `run_simulation.m`: The main MATLAB script that runs the PSO algorithm, evaluates fitness, and plots results.
- `simulation.mdl`: Simulink model used for simulating the system response with the current PID parameters.
- `target_function.m`: A fitness function embedded within the script, responsible for evaluating each set of PID parameters.
- `plots/`: Folder where convergence plots are saved.
- `results/`: Folder where optimized PID parameters and associated data are saved as `.mat` files.

### Prerequisites

- MATLAB with Simulink
- Basic understanding of control systems and optimization techniques

### Running the Project

1. Clone this repository and navigate to the project directory.
2. Run the main script:
   ```matlab
   run('run_simulation.m')
   ```
3. The script performs PSO-based optimization, simulates each configuration, and logs the best PID parameters. Results are saved to `results/optimized_pid_results.mat`, and plots of fitness and PID convergence are saved in the `plots/` folder.

### Output

The script outputs the optimized PID parameters, best fitness achieved, and plots for:
- **Best Fitness Over Iterations**: Convergence of the best fitness value across iterations.
- **PID Parameter Convergence**: Evolution of each PID parameter (Kp, Ki, Kd) over iterations.

### Details of the PSO Algorithm Implementation

- **Particles and Iterations**: The swarm consists of 50 particles, each representing a candidate solution in the form of PID parameters. The PSO runs for 10 iterations, during which particles update their positions based on individual and global best values.
- **Fitness Evaluation**: The fitness function is defined in `tracklsq`, combining error and overshoot as criteria to be minimized.
- **Velocity and Position Updates**: Each particle’s velocity is updated using PSO coefficients—cognitive, social, and inertia terms—balancing exploration and convergence.

