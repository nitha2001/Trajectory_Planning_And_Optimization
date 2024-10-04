# Trajectory_Planning_And_Optimization
Implementation of **iLQR** for a 2D quadrotor system, **direct collocation** methods for trajectory planning, and trajectory optimization for a planar arm system with a rigidly attached ball.

## Project Structure

The project is organized into several Python files, each addressing a specific aspect of the problem set:

- `iLQR.py`: Contains the implementation of the iterative Linear Quadratic Regulator (iLQR) algorithm.
- `find_throwing_trajectory.py`: Implements the direct collocation method for optimizing the trajectory of a planar arm system.
- `kinematic_constraints.py`: Contains the constraints related to the kinematics of the system.
- `dynamics_constraints.py`: Implements the dynamics constraints for the robotic system.

## ILQR Implementation
iLQR Implementation
The iLQR algorithm is implemented in `iLQR.py`, where we focus on optimizing the control trajectory for a 2D quadrotor system. Key components of the implementation include:

**Backward Pass:**

- Implemented using the calculated derivatives of the running and terminal costs.
- The gradient and Hessian of the cost function are computed using the following methods:
    - `grad_running_cost()`
    - `hess_running_cost()`
    - `grad_terminal_cost()`
    - `hess_terminal_cost()`
- The backward pass is structured to compute the updates for **𝑔**𝑘 and **H**k as described in the problem set, ensuring numerical stability during optimization.

**Forward Pass:**
- The forward pass updates the trajectory based on the newly calculated control law:
    - Initializes with the previously estimated trajectory.
    - Updates state and control inputs using the computed **K**k and **d**k.
    - The parameter 𝛼 controls the step size for stability.

**Key Functions**

- `backward_pass()`: Computes the necessary gradients and Hessians, updating control inputs.
- `forward_pass()`: Simulates the system dynamics forward in time using the updated control law.

## Direct Collocation Implementation

The Direct Collocation method approximates the state and input trajectories using cubic and linear splines, respectively. The implementation details include:

**Spline Coefficients:**

- Implemented the calculation of spline coefficients to satisfy the dynamics at the endpoints and midpoints.
- Midpoint dynamics are enforced by adding constraints based on the dynamics.
  
**Trajectory Optimization:**

- Utilizes a nonlinear program to minimize input effort while adhering to constraints on torque, velocity, and end effector position.
- Uses `pydrake` for constraint management and optimization formulation.
  
**Key Functions**
`add_constraints()`: Enforces kinematic and dynamic constraints on the spline trajectory.
`optimize_trajectory()`: Solves the optimization problem for the planar arm, ensuring that the trajectory satisfies all defined constraints.

## Planar Arm Trajectory Optimization

In this section, we implement a trajectory optimization for a planar arm system with a ball at the end effector. The key tasks include:

**Setting Up Constraints:**

- Implemented kinematic constraints in kinematic_constraints.py to ensure that the ball lands at the desired distance from the robot's base.
- Dynamic constraints are defined in dynamics_constraints.py to model the physical behavior of the system.
  
**Trajectory Execution:**

- The trajectory is optimized to minimize input efforts while satisfying constraints on final position and velocity.
  
**Key Functions**

- `add_landing_constraints()`: Enforces constraints related to the landing position of the ball after release.
- `add_final_state_constraints()`: Ensures the final configuration meets the required parameters.


## Results
![download](https://github.com/user-attachments/assets/a2b598ea-00a5-4207-a8a8-1a0a32729f33)

![image](https://github.com/user-attachments/assets/4a9e009f-52bd-4986-9c0e-84a7330ecb12)
