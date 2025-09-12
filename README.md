# A differential drive robot with reference tracking while maintaining a safe distance from moving obstacles.

**NOTES**

- Even though CasADi seems the way to go for this project, SciPy is preferred in order to direct control over all variables of the optimal control problem at any given time.
- Obstacle avoidance is **not** yet implemented. There seems to be an issue of the SLSQP solver violating inequality constraints, but this needs examination with a different formulation/library since only SLSQP and trust-constr(really slow) can be used with SciPy.
---


A diff. drive robot is described by the bellow nonlinear diffential equations. The robot state is described by $$x_{\textrm{k}},\quad  y_{\textrm{k}}, \quad \theta_{\textrm{k}}$$ representing current position on the x-axis, y-axis and the heading angle on the global axis frame respectively.

$$
\Large
\begin{aligned}
p_1 &= \frac{v_{\textrm{k}}}{w_{\textrm{k}}}\\
x_{\textrm{k+1}} &= x_{\textrm{k}} - p_1 \, \sin(\theta_{\textrm{k}}) + p_1 \, \sin(\theta_{\textrm{k}} + \omega_{\textrm{k}} \, \Delta t) \\
y_{\textrm{k+1}} &= y_{\textrm{k}} + p_1 \, \cos(\theta_{\textrm{k}}) - p_1 \, \cos(\theta_{\textrm{k}} + \omega_{\textrm{k}} \, \Delta t) \\
\theta_{\textrm{k+1}} &= \theta_{\textrm{k}} + \omega_{\textrm{k}} \, \Delta t
\end{aligned}
$$

# Reference Tracking
A nonlinear Model Predictive Controller is used to drive the robot's current position to the desired position. The MPC is designed with the direct multiple shooting scheme leading to a more stable optimization process. For the optimizer we will be using SciPy's minimize method.

By providing the MPC with an objective function of the form:
$$J = (x - x_{\textrm{ref}}) Q (x - x_{\textrm{ref}})^T + u R u^T
$$

We impose a soft constraint to our problem which is to minimize the distance of our current state x to xref by also minimizing our control effort u.

No hard constraints are imposed for the reference tracking task!

# Obstacle Avoidance
For obstacle avoidance we can impose a hard constraint to the MPC for our robot's center to always be outside of a given safety zone like:
$$
\sqrt{(x_{\textrm{robot}} - x_{\textrm{obs}})**2 + (y_{\textrm{robot}} - y_{\textrm{obs}})**2} + d
$$
with d being a safety distance and x_obs, y_obs being the closest point of the obstacle to our robot. (we find that with pygame's clipline)
