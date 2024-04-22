import numpy as np

def cbf_control(x, x_obs, y_obs, u_nominal, D=0.2, alpha=1.0):
    """
    Compute control input u using CBF for obstacle avoidance.
    x: Current state [x, y, theta].
    x_obs, y_obs: Obstacle position.
    u_nominal: Nominal control inputs [cmd.linear.x, cmd.angular.z] without considering obstacles.
    D: Safety distance.
    alpha: Adjustment factor for the control correction.
    """
    # Safety condition and its gradient
    h_val = (x[0] - x_obs)**2 + (x[1] - y_obs)**2 - D**2
    dh_dx = np.array([2*(x[0] - x_obs), 2*(x[1] - y_obs), 0])
    
    f_x = 0.0
    g_x = np.array([[np.cos(x[2]), 0], [np.sin(x[2]), 0], [0, 1]])
    
    # Lie derivative of h with respect to the dynamics
    # Lf_h = np.dot(dh_dx, f_x)
    Lg_h = np.dot(dh_dx, g_x)

    # Adjust control to satisfy the safety condition
    delta_u = -alpha*h_val*np.array([1/Lg_h[0], 1/Lg_h[1]])
    # print(f'delta_u={delta_u}')
    
    # Filter the inf data
    for i in range(2):
        if delta_u[i] >= 0.0:
            delta_u[i] = 0.0
        if np.abs(delta_u[i]) == np.inf:
            delta_u[i] = 0.0
    
    u_safe = u_nominal + delta_u
    
    return u_safe


# # Examples
# x = np.array([1.5, 0.7, np.pi/2])  # Robot's initial state
# x_obs = np.array([1.5, 0.6])  # Obstacle position
# u_nominal = np.array([0.2, 0.01])  # Nominal control input (v, omega)
# # Compute safe control input
# u_safe = cbf_control(x, x_obs[0], x_obs[1], u_nominal)
# print(f"Safe control input: {u_safe}")

