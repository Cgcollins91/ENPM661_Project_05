# %% Chris Collins UID 110697305 Homework 2, ENPM 662


from sympy.utilities.lambdify import lambdify
from math import cos, sin
import matplotlib.pyplot as plt
import numpy as np
import sympy as sp
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from mpl_toolkits.mplot3d import Axes3D
import plotly.graph_objects as go
import math


def get_symbolic_DH_matrix(i):
    """ Get the symbolic DH matrix for the ith joint of the robot"""
    theta = sp.symbols(f'theta_{i}')

    if i == '1' or i == '2':
        theta = theta - sp.pi / 2  # Subtract pi/2 for joint 1 and 2

    elif i == '3':
        theta = theta + sp.pi      # Subtract pi for joint 3

    elif i == '4' or i == '5':
        theta = theta + sp.pi / 2 # Add pi/2 for joint 4 & 5

    alpha = sp.symbols(f'alpha_{i}')
    a     = sp.symbols(f'a_{i}')
    d     = sp.symbols(f'd_{i}')
    
    return sp.Matrix([
                     [ sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha),   a*sp.cos(theta) ],
                     [ sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha),   a*sp.sin(theta) ],
                     [             0,                sp.sin(alpha),                sp.cos(alpha),   d               ],
                     [             0,                            0,                            0,   1               ]
                     ])


def get_sympy_inputs(theta_1=0, theta_2=0, theta_3=0, theta_4=0, theta_5=0, theta_6=0):
    """ 
    Get inputs to sympy transformation matrices, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 are variable, 
    all else are fixed
    """
    
    values          = {}
    theta_variables = []
    indexes         = ['1',       '2',    '3',     '4',      '5',       '6']

    a     = [    0.0,         0.85,      0.75,         0,         0,         0 ]
    alpha = [ -np.pi/2,     np.pi,         0,    -np.pi/2,   np.pi/2,         0 ]
    d     = [     0.05,      0.05,        0.1,      0.05,      0.2,       0.05 ]
    theta = [  theta_1,   theta_2,    theta_3,   theta_4,  theta_5,   theta_6 ]
 

    for index, i in zip(indexes, range(1, 7)):
        values[sp.symbols(f'theta_{index}')] = theta[i-1]
        theta_variables.append(sp.symbols(f'theta_{index}'))
        values[sp.symbols(f'alpha_{index}')] = alpha[i-1]
        values[sp.symbols(f'a_{index}')]     = a[i-1]
        values[sp.symbols(f'd_{index}')]     = d[i-1]

    theta_variables = [theta for theta in theta_variables if isinstance(theta, sp.Basic)]

    return values, theta_variables


def get_sympy_thetas(theta_1=0, theta_2=0, theta_3=0, theta_4=0, theta_5=0, theta_6=0):
    """Get theta (Variable) inputs to sympy transformation matrices, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 
        All other variables are fixed
    """
    values = {}
    thetas = [ theta_1,        theta_2,             theta_3,        theta_4,  theta_5,   theta_6] 
    indexes = ['1',        '2',              '3',         '4',      '5',       '6']

    for index, i in zip(indexes, range(1, 7)):
        values[sp.symbols(f'theta_{index}')] = thetas[i-1]

    return values


def full_forward_kinematics_numeric(T_matrices, theta_current):
    """ Compute the forward kinematics of the robot numerically given the symbolic transformation matrices and joint angles
    Parameters: T_matrices:    List of symbolic transformation matrices
                theta_current: Dictionary of joint angles

    Returns: origins:       List of origins of the robot for the case configuration
            origins_padded: List of origins of the robot for the case configuration with the homogeneous coordinate 
            T_matrices:     List of symbolic transformation matrices
    """
    origin = sp.Matrix([0, 0, 0, 1])

    T01sub, T02sub,  T03sub, T04sub, T05sub, T06sub = [T.subs(theta_current) for T in T_matrices]

    o1,  o2,  o3,  o4, o5, o6 = [T*origin for T in [T01sub,  T02sub, T03sub,  T04sub, T05sub, T06sub]]

    origin, o1, o2,  o3,  o4, o5, o6   = [np.array(origin).astype(np.float64)] + [np.array(o).astype(np.float64) for o in [o1,  o2,  o3,  o4, o5, o6 ]]
    origins_padded                     = [origin, o1,  o2,  o3,  o4, o5, o6 ]

    origin, o1,  o2,  o3,  o4, o5, o6   = [origin[:3]] + [o[:3] for o in [o1,  o2,  o3,  o4, o5, o6 ]]
    origins    = [origin, o1,  o2, o3, o4, o5, o6 ]
    T_matrices = [T01sub,  T02sub,  T03sub,  T04sub, T05sub, T06sub]

    return origins, origins_padded, T_matrices


def run_test_case(theta_1=0, theta_2=0, theta_3=0, theta_4=0, theta_5=0, theta_6=0, T_home_matrices=[], case_title="Test Case", print_origins=True):
    """ Run test case with given joint angles then plot origins of robot for home and case configurations
    Parmeters: theta_1, theta_2, theta_3, theta_4, theta_5, theta_6:   Joint angles in radians
               T_home_matrices:   List of symbolic transformation matrices for the home configuration
                case_title:       Title of the test case
                print_origins:    Boolean to print the origins of the robot for the home and case configurations
    Returns:   home_origins:     List of origins of the robot for the home configuration
                case_origins:     List of origins of the robot for the case configuration
                  """
    origin = sp.Matrix([0, 0, 0, 1])
    fig    = go.Figure()


    theta_current, theta_variables           = get_sympy_inputs(
        theta_1=0, theta_2=0, theta_3=0, 
        theta_4=0, theta_5=0, theta_6=0
        )
    home_origins, origins_padded, T_matrices = full_forward_kinematics_numeric(T_home_matrices, theta_current)
    

    theta_current, theta_variables           = get_sympy_inputs(
        theta_1=theta_1, theta_2=theta_2, theta_3=theta_3,
        theta_4=theta_4, theta_5=theta_5, theta_6=theta_6)
    case_origins, origins_padded, T_matrices = full_forward_kinematics_numeric(T_home_matrices, theta_current)


    home_origins = [[float(coord) for coord in origin[:3]] for origin in home_origins]  # Convert only x, y, z
    case_origins = [[float(coord) for coord in origin[:3]] for origin in case_origins]  # Convert only x, y, z
    xx, yy, zz = [], [], []

    # Plot each origin as a point and draw a line between consecutive origins if desired
    for i, origin in enumerate(home_origins):
        xx.append(origin[0]), yy.append(origin[1]), zz.append(origin[2])   
        # Plot the origin as a point
        fig.add_trace(go.Scatter3d(
            x=[origin[0]], y=[origin[1]], z=[origin[2]],
            mode='markers+text',
            marker=dict(size=5, color='blue'),
            text=f'Home O {i}',
            name=f'Home O {i}'
        ))

        # Draw a line connecting the current origin to the next one
        if i < len(home_origins) - 1:
            next_origin  = home_origins[i + 1]
            prior_origin = home_origins[i]
            
            fig.add_trace(go.Scatter3d(
                x=[prior_origin[0], next_origin[0]],
                y=[prior_origin[1], next_origin[1]],
                z=[prior_origin[2], next_origin[2]],
                mode='lines',
                name=f'Home O {i}',
                line=dict(color='blue', width=3),
                showlegend=False
            ))

    # Plot each origin as a point and draw a line between consecutive origins if desired
    for i, origin in enumerate(case_origins):
        xx.append(origin[0]), yy.append(origin[1]), zz.append(origin[2])   
        # Plot the origin as a point
        fig.add_trace(go.Scatter3d(
            x=[origin[0]], y=[origin[1]], z=[origin[2]],
            mode='markers+text',
            marker=dict(size=5, color='orange'),
            text=f'Case O {i}',
            name=f'Case O {i}'
        ))

        # Draw a line connecting the current origin to the next one
        if i < len(case_origins) - 1:
            next_origin  = case_origins[i + 1]
            prior_origin = case_origins[i]
            
            fig.add_trace(go.Scatter3d(
                x=[prior_origin[0], next_origin[0]],
                y=[prior_origin[1], next_origin[1]],
                z=[prior_origin[2], next_origin[2]],
                mode='lines',
                line=dict(color='orange', width=3),
                showlegend=False
            ))

    min_x, max_x = min(xx), max(xx)
    min_y, max_y = min(yy), max(yy)
    min_z, max_z = min(zz), max(zz)
    fig.update_layout(
        title=case_title,
        scene=dict(
            xaxis_title="X",
            yaxis_title="Y",
            zaxis_title="Z",
            xaxis=dict(title='X', range=[min_x-.2, max_x+.2]),
            yaxis=dict(title='Y', range=[min_y, max_y]),
            zaxis=dict(title='Z', range=[min_z, max_z]),
            aspectmode='cube'
        )
    )
    fig.update_layout(
    scene_camera=dict(
        eye=dict(x=1.25, y=1.25, z=1.25)  # Equal distance along each axis
    )
    )

    fig.show()


    
    
    return home_origins, case_origins


def compute_jacobian_numeric(J, theta_current):
    # Convert symbolic Jacobian to a numerical function
    J_subs    = J.subs(theta_current).evalf()
    J_numeric = np.array(J_subs).astype(np.float64)
    return J_numeric


def forward_kinematics_numeric(T06, theta_current):
    # Compute the end-effector position numerically
    T06_subs          = T06.subs(theta_current).evalf()
    T_numeric         = np.array(T06_subs).astype(np.float64)
    position_numeric  = T_numeric[:3, 3]
    return position_numeric, T_numeric


def compute_orientation_error(R_desired, R_current):
    """
    Compute orientation error between desired and current rotation matrices 
    (NOT USED IN THIS HOMEWORK, BUT WAS UTILIZED FOR CONVERGENCE TESTING)

    Parameters:
    - R_desired: Desired rotation matrix (3x3 numpy array)
    - R_current: Current rotation matrix (3x3 numpy array)

    Returns:
    - error_orientation: Orientation error vector (numpy array of shape (3,))
    """
    # Compute the relative rotation matrix
    R_error = R_desired @ R_current.T  # Equivalent to R_desired * R_current^-1

    # Compute the angle of rotation using the trace of the relative rotation matrix
    cos_theta = (np.trace(R_error) - 1) / 2.0

    # Numerical stability check to keep cos_theta within valid range [-1, 1]
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)

    # Check if the angle is negligible
    if np.isclose(theta, 0):
        # If the angle is very small, the orientation error is zero
        error_orientation = np.zeros(3)

    else:
        # Compute the skew-symmetric part of R_error
        skew_symmetric = (R_error - R_error.T) / (2 * np.sin(theta))

        # Extract the rotation axis from the skew-symmetric matrix
        rx = skew_symmetric[2, 1]
        ry = skew_symmetric[0, 2]
        rz = skew_symmetric[1, 0]
        rotation_axis = np.array([rx, ry, rz])

        # Orientation error vector is the rotation axis scaled by the angle
        error_orientation = theta * rotation_axis

    return error_orientation


def reduce_angle(angle):
    """ This function reduces an angle to be within -2pi and 2pi """
    angle %= 2 * math.pi # Reduce the angle modulo 2pi
    
    # Adjust to be within -2pi and 2pi
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    
    return angle


def get_distances_between_joints(origins):
    ''' This function computes the relative and absolute distances between the joints'''
    xj, yj, zj = [], [], []
    xoj, yoj, zoj = [], [], []
    xoji, yoji, zoji = 0, 0, 0

    # This gives the relative position of each joint with respect to the previous joint
    for i in range(1, 7):
        prior_joint = origins[i-1]
        joint       = origins[i]
        delta_z     =  joint[3]-prior_joint[3]
        delta_y     =  joint[2]-prior_joint[2]
        delta_x     =  joint[1]-prior_joint[1]
        xj.append(delta_x), yj.append(delta_y), zj.append(delta_z)

    # This gives the absolute position of each joint with respect to the base frame
    for xj, yj, zj in zip(xj, yj, zj):
        xoji += xj
        yoji += yj
        zoji += zj

        xoj.append(xoji), yoj.append(yoji), zoj.append(zoji)

    return xj, yj, zj, xoj, yoj, zoj


def get_thetas(theta_current):
    theta_1  = theta_current[sp.symbols('theta_1')]

    theta_2  = theta_current[sp.symbols('theta_2')]

    theta_3  = theta_current[sp.symbols('theta_3')]

    theta_4  = theta_current[sp.symbols('theta_4')]
    theta_5  = theta_current[sp.symbols('theta_5')]
    theta_6  = theta_current[sp.symbols('theta_6')]

    return [theta_1,  theta_2,  theta_3, theta_4, theta_5, theta_6]


def plot_test_cases(T_home_matrices):
    # Case 1, theta_1 = pi/2
    theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = np.pi/2, 0, 0, 0, 0, 0
    case_title = "Case 1, theta_1 = pi/2"
    home_origins, case_origins = run_test_case(theta_1=theta_1, theta_2=theta_2, theta_3=theta_3, theta_4=theta_4, theta_5=theta_5, theta_6=theta_6, T_home_matrices=T_home_matrices, case_title=case_title, print_origins=False)

    # Case 2, theta_2 = pi/2
    theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = 0, np.pi/2, 0, 0, 0, 0
    case_title = "Case 2, theta_2 = pi/2"
    home_origins, case_origins = run_test_case(theta_1=theta_1, theta_2=theta_2, theta_3=theta_3, theta_4=theta_4, theta_5=theta_5, theta_6=theta_6, T_home_matrices=T_home_matrices, case_title=case_title, print_origins=False)

    #   Case 3, theta_3 = pi/2
    theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = 0, 0, np.pi/2, 0, 0, 0
    case_title = "Case 3, theta_3 = pi/2"
    home_origins, case_origins = run_test_case(theta_1=theta_1, theta_2=theta_2, theta_3=theta_3, theta_4=theta_4, theta_5=theta_5, theta_6=theta_6, T_home_matrices=T_home_matrices, case_title=case_title, print_origins=False)

    # Case 5, theta_4 = pi/2
    theta_1, theta_2, theta_3, theta_4, theta_5, theta_6  = 0, 0, 0, np.pi/2, 0, 0
    case_title = "Case 4, theta_4 = pi/2"   
    home_origins, case_origins = run_test_case(theta_1=theta_1, theta_2=theta_2, theta_3=theta_3, theta_4=theta_4, theta_5=theta_5, theta_6=theta_6, T_home_matrices=T_home_matrices, case_title=case_title, print_origins=False)

    # Case 5, theta_5 = pi/2
    theta_1, theta_2, theta_3, theta_4,  theta_5, theta_6 = 0, 0, 0, 0, np.pi/2, 0
    case_title = "Case 5, theta_5 = pi/2"   
    home_origins, case_origins = run_test_case(theta_1=theta_1, theta_2=theta_2, theta_3=theta_3, theta_4=theta_4, theta_5=theta_5, theta_6=theta_6, T_home_matrices=T_home_matrices, case_title=case_title, print_origins=False)


def plot_joints(t, q_list, p_curr_list, q_dot_list, error_list, joint_torques):
    theta_1 = [row[0] for row in q_list]
    theta_2 = [row[1] for row in q_list]
    theta_3 = [row[2] for row in q_list]
    theta_4 = [row[3] for row in q_list]
    theta_5 = [row[4] for row in q_list]
    theta_6 = [row[5] for row in q_list]
    x_error = [row[0] for row in error_list]
    y_error = [row[1] for row in error_list]
    z_error = [row[2] for row in error_list]

    x       = [row[0] for row in p_curr_list]
    y       = [row[1] for row in p_curr_list]
    z       = [row[2] for row in p_curr_list]

    theta_1_dot = [row[0] for row in q_dot_list]
    theta_2_dot = [row[1] for row in q_dot_list]
    theta_3_dot = [row[2] for row in q_dot_list]
    theta_4_dot = [row[3] for row in q_dot_list]
    theta_5_dot = [row[4] for row in q_dot_list]
    theta_6_dot = [row[5] for row in q_dot_list]


    theta_1     = [math.degrees(angle) for angle in theta_1]
    theta_2     = [math.degrees(angle) for angle in theta_2]
    theta_3     = [math.degrees(angle) for angle in theta_3]
    theta_4     = [math.degrees(angle) for angle in theta_4]
    theta_5     = [math.degrees(angle) for angle in theta_5]
    theta_6     = [math.degrees(angle) for angle in theta_6]

    theta_1_dot = [math.degrees(angle) for angle in theta_1_dot]
    theta_2_dot = [math.degrees(angle) for angle in theta_2_dot]
    theta_3_dot = [math.degrees(angle) for angle in theta_3_dot]
    theta_4_dot = [math.degrees(angle) for angle in theta_4_dot]
    theta_5_dot = [math.degrees(angle) for angle in theta_5_dot]
    theta_6_dot = [math.degrees(angle) for angle in theta_6_dot]

    t1 = [row[0] for row in joint_torques]
    t2 = [row[1] for row in joint_torques]
    t3 = [row[2] for row in joint_torques]
    t4 = [row[3] for row in joint_torques]
    t5 = [row[4] for row in joint_torques]
    t6 = [row[5] for row in joint_torques]

    
    # Plot X, Y, Z

    plt.figure(figsize=(12, 8))
    plt.subplot(3, 3, 1)
    plt.plot(t, x,  color='purple')
    plt.xlabel('Time (s)')
    plt.ylabel('X (m)')
    plt.title('End Effector X position')
    plt.grid(True)

    plt.subplot(3, 3, 2)
    plt.plot(t, y,  color='green')
    plt.ylabel('End Effector Y (m)')
    plt.xlabel('Time (s)')
    plt.title('End Effector Y position')
    plt.grid(True)

    plt.subplot(3, 3, 3)
    plt.plot(t, z,  color='blue')
    plt.ylabel('End Effector Z (m)')
    plt.xlabel('Time (s)')
    plt.title('End Effector Z position')
    plt.grid(True)

    plt.subplot(3, 3, 4)
    plt.plot(t, x_error,  color='purple')
    plt.ylabel('X Error (m)')
    plt.xlabel('Time (s)')
    plt.title('X Error vs. T')
    plt.grid(True)

    plt.subplot(3, 3, 5)
    plt.plot(t, y_error,  color='green')
    plt.ylabel('Y Error (m)')
    plt.xlabel('Time (s)')
    plt.title('Y Error vs. T')
    plt.grid(True)

    plt.subplot(3, 3, 6)
    plt.plot(t, z_error,  color='blue')
    plt.ylabel('Z Error (m)')
    plt.xlabel('Time (s)')
    plt.title('Z Error vs. T')
    plt.grid(True)

    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(12, 8))
    plt.subplot(3, 3, 1)
    plt.plot(x, y,  color='black')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Effector X vs. Y')
    plt.grid(True)

    plt.subplot(3, 3, 2)
    plt.plot(x, z,  color='black')
    plt.xlabel('X (m)')
    plt.ylabel('Z (m)')
    plt.title('Effector X vs. Z ')
    plt.grid(True)

    plt.subplot(3, 3, 3)
    plt.plot(y, z,  color='black')
    plt.xlabel('Y (m)')
    plt.ylabel('Z (m)')
    plt.title('Effector Y vs. Z')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(12, 8))

    plt.subplot(3, 3, 1)
    plt.plot(t, theta_1, label='Theta 1', color='blue')
    plt.ylabel('Angle (degrees)')
    plt.title('Theta 1 Angle')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 3, 2)
    plt.plot(t, theta_2, label='Theta 2', color='orange')
    plt.ylabel('Angle (degrees)')
    plt.legend()
    plt.title('Theta 2 Angle')
    plt.grid(True)

    plt.subplot(3, 3, 3)
    plt.plot(t, theta_3, label='Theta 3', color='green')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.title('Theta 3 Angle')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 3, 4)
    plt.plot(t, theta_4, label='Theta 4', color='magenta')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.title('Theta 4 Angle')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 3, 5)
    plt.plot(t, theta_5, label='Theta 5', color='purple')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.title('Theta 5 Angle')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 3, 6)
    plt.plot(t, theta_6, label='Theta 6', color='black')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.title('Theta 6 Angle')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


    plt.figure(figsize=(12, 8))

    plt.subplot(3, 3, 1)
    plt.plot(t, t1, label='Joint 1 Torque', color='blue')
    plt.ylabel('Torque (N-M)')
    plt.title('Joint 1 Torque')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 3, 2)
    plt.plot(t, t2, label='Joint 2 Torque', color='orange')
    plt.ylabel('Torque (N-M)')
    plt.title('Joint 2 Torque')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 3, 3)
    plt.plot(t, t3, label='Joint 3 Torque', color='green')
    plt.ylabel('Torque (N-M)')
    plt.title('Joint 3 Torque')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 3, 4)
    plt.plot(t, t4, label='Joint 4 Torque', color='magenta')
    plt.ylabel('Torque (N-M)')
    plt.title('Joint 4 Torque')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 3, 5)
    plt.plot(t, t5, label='Joint 5 Torque', color='purple')
    plt.ylabel('Torque (N-M)')
    plt.title('Joint 5 Torque')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 3, 6)
    plt.plot(t, t6, label='Joint 6 Torque', color='black')
    plt.ylabel('Torque (N-M)')
    plt.title('Joint 6 Torque')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


    plt.figure(figsize=(12, 8))

    plt.subplot(3, 3, 1)
    plt.plot(t, theta_1_dot, label='Theta 1 Velocity', color='blue')
    plt.ylabel('Velocity (degrees/s)')
    plt.title('Theta 1 Velocity')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 3, 2)
    plt.plot(t, theta_2_dot, label='Theta 2 Velocity', color='orange')
    plt.ylabel('Velocity (degrees/s)')
    plt.title('Theta 2 Velocity')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 3, 3)
    plt.plot(t, theta_3_dot, label='Theta 3 Veloctity', color='green')
    plt.ylabel('Velocity (degrees/s)')
    plt.title('Theta 3 Velocity')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 3, 4)
    plt.plot(t, theta_4_dot, label='Theta 4 Velocity', color='magenta')
    plt.ylabel('Velocity (degrees/s)')
    plt.title('Theta 4 Velocity')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 3, 5)
    plt.plot(t, theta_5_dot, label='Theta 5 Velocity', color='purple')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (degrees/s)')
    plt.title('Theta 5 Velocity')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 3, 6)
    plt.plot(t, theta_6_dot, label='Theta 6 Velocity', color='black')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (degrees/s)')
    plt.title('Theta 6 Velocity')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

def sub_fixed_and_lambdify(mat_in, start_values_fixed, theta_variables):
    ''' This function takes in the symbolic transformation matrices and Jacobian, and returns lambdified functions'''
    
    mat_in_subs   = mat_in.subs(start_values_fixed)

    #  Flatten Jacobian for lambdification
    mat_in_flat = mat_in_subs.reshape(mat_in_subs.shape[0]*mat_in_subs.shape[1], 1)
    mat_in_func = sp.lambdify(theta_variables, mat_in_flat, modules='numpy')


    return mat_in_func, mat_in_subs

def get_lamdified(J, J_CoM, joint_torques, T_home_matrices, start_values_fixed, theta_variables):
    ''' This function takes in the symbolic transformation matrices and Jacobian, and returns lambdified functions'''
    [T01, T02, T03, T04, T05, T06] = T_home_matrices
    J_func, J_subs         = sub_fixed_and_lambdify(J,   start_values_fixed, theta_variables)
    J_CoM_func, J_CoM_subs = sub_fixed_and_lambdify(J_CoM, start_values_fixed, theta_variables)
    T01_func, T01_subs     = sub_fixed_and_lambdify(T01, start_values_fixed, theta_variables)
    T02_func, T02_subs     = sub_fixed_and_lambdify(T02, start_values_fixed, theta_variables)
    T03_func, T03_subs     = sub_fixed_and_lambdify(T03, start_values_fixed, theta_variables)
    T04_func, T04_subs     = sub_fixed_and_lambdify(T04, start_values_fixed, theta_variables)
    T05_func, T05_subs     = sub_fixed_and_lambdify(T05, start_values_fixed, theta_variables)
    T06_func, T06_subs     = sub_fixed_and_lambdify(T06, start_values_fixed, theta_variables)

    joint_torques_func, joint_torques_subs = sub_fixed_and_lambdify(joint_torques, start_values_fixed, theta_variables)

    func_mats = { 'J_func': J_func,    'J_CoM_func': J_CoM_func,
                 'joint_torques_func': joint_torques_func,
                 'T01_func': T01_func, 'T02_func'  : T02_func, 
                 'T03_func': T03_func, 'T04_func'  : T04_func, 
                 'T05_func': T05_func, 'T06_func'  : T06_func}
    
    subs_mats = { 'J_subs': J_subs,    'J_CoM_subs': J_CoM_subs,
                'joint_torques_subs': joint_torques_subs,
                 'T01_subs': T01_subs, 'T02_subs'  : T02_subs, 
                 'T03_subs': T03_subs, 'T04_subs'  : T04_subs, 
                 'T05_subs': T05_subs, 'T06_subs'  : T06_subs}

    return func_mats, subs_mats

def get_inertia_tensor(i):
    # NOT USED IN HOMEWORK
    # Link 1
    if i == 1:
        Ixx = 21031638.84	
        Ixy = 1.96	
        Ixz = 7.27
        Iyx = 1.96	
        Iyy = 20739417.85	
        Iyz = -1270611.01
        Izx = 7.27	
        Izy = -1270611.01	
        Izz = 1209353.75
    
    elif i == 2: # Link 2
        Ixx = 168766539.22	
        Ixy = -616.13	
        Ixz = 2144.80
        Iyx = -616.13	
        Iyy = 141311550.02	
        Iyz = -57583858.50
        Izx = 2144.80	
        Izy = -57583858.50
        Izz = 29041369.78

    elif i == 3:
        # Link 3
        Ixx = 237736457.80
        Ixy = -42.62
        Ixz = 349.73
        Iyx = -42.62
        Iyy = 236687453.94
        Iyz = -13865529.11
        Izx = 349.73
        Izy = -13865529.11
        Izz = 1495860.09

    elif i == 4:
        #Link 4
        Ixx = 144611320.32
        Ixy = 3.69
        Ixz = 0.32
        Iyx = 3.69	
        Iyy = 139216600.79
        Iyz = -27063652.79
        Izx = 0.32	
        Izy = -27063652.79
        Izz = 5587457.88

    elif i == 5:
        #Link 5
        Ixx = 156009135.98
        Ixy = 1.06
        Ixz = 3.34
        Iyx = 1.06
        Iyy = 150468519.15
        Iyz = -28605700.82
        Izx = 3.34
        Izy = -28605700.82
        Izz = 5704521.06

    elif i == 6:
        #Link 6
        Ixx = 76033893.64
        Ixy = -1.93
        Ixz = 6.77
        Iyx = -1.93
        Iyy = 70332861.38
        Iyz = -20030454.36
        Izx = 6.77
        Izy = -20030454.36
        Izz = 5775378.88
    else:
        print("Invalid Link Number")

    # Convert from grams*mm^2 to kg * m^2
    Ixx, Ixy, Ixz, Iyx, Iyy, Iyz, Izx, Izy, Izz = Ixx/10**9, Ixy/10**9, Ixz/10**9, Iyx/10**9, Iyy/10**9, Iyz/10**9, Izx/10**9, Izy/10**9, Izz/10**9
    return sp.Matrix([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]])

def get_intertial_matrices():
    # NOT USED IN HOMEWORK
    inertial_mats = []
    for i in range(1,7):
        inertial_mats.append(get_inertia_tensor(i))
    return inertial_mats

def get_D_of_q(inertial_mats, J, theta_current, T_i_com, m):
    # NOT USED IN HOMEWORK
    D = 0
    for i in range(6):
        ith_column   = J.subs(theta_current)[:, i]
        T_i_com_subs = T_i_com[i].subs(theta_current)
        inerital_mat = inertial_mats[i]
        v_com_i      = sp.Matrix([ith_column[0], ith_column[1], ith_column[2]])
        w_com_i      = sp.Matrix([ith_column[3], ith_column[4], ith_column[5]])
        
        R_0i         = T_i_com_subs[:3, :3]
        I_base       = R_0i * inerital_mat * R_0i.T
        omega_i      = (1/2) * w_com_i.T * I_base * w_com_i
        linear_i     =(1/2) * m[i]*(v_com_i.T * v_com_i)
        D           += (omega_i + linear_i)[0]
    return D

def get_robot_transformations():
    ''' This function computes the symbolic transformation and Jacobian matrices for each joint according 
    to our DH parameters'''

    #  Get Synbolic Transformation Matrices for each joint
    T01   = get_symbolic_DH_matrix('1')
    T12   = get_symbolic_DH_matrix('2')
    T23   = get_symbolic_DH_matrix('3')
    T34   = get_symbolic_DH_matrix('4')
    T45   = get_symbolic_DH_matrix('5')
    T56   = get_symbolic_DH_matrix('6')

    # Compute the symbolic transformation matrices for each joint relative to the base frame
    T02  = T01*T12
    T03  = T02*T23
    T04  = T03*T34
    T05  = T04*T45
    T06  = T05*T56

    T_home_matrices = [T01,  T02, T03, T04, T05, T06]
    o0              = sp.Matrix([0, 0, 0, 1])

    # Origins of each joint
    home_origins_sym = [o0] + [T * o0 for T in T_home_matrices]

    # Z vectors of each joint
    Z0        = sp.Matrix([0, 0, 1])
    Z_vectors = [Z0] + [T[:3, 2] for T in T_home_matrices]

    # Compute Jacobian Using the First Method leveraging each joint velocity is function of r*theta_dot, where r can be 
    # calculated from the cross product of Z and L (O_n - O_i-1)
    J = []
    for i in range(1, 7):
        Z_i_1   = Z_vectors[i-1]
        o_i_1   = home_origins_sym[i-1][0:3, 0]
        L       = home_origins_sym[6][0:3, 0] - o_i_1
        J_i_0   = Z_i_1.cross(L)
        J_i     = J_i_0.col_join(Z_i_1)
        J.append(J_i)

    J  = sp.Matrix.hstack(*J)

    return T_home_matrices, J

def get_CoM_parameters(T_home_matrices):
    ''' This function computes the symbolic transformation and Jacobian matrices for our Center of Masses'''
    print("COM based on hw3 and wrong DH parameters")
    r_1   = sp.Matrix([          0,   0,    -.015, 1 ]) # Frame 1 Coordinates
    r_2   = sp.Matrix([  2/3*.7371,   0,   -.1723, 1 ]) # Frame 2 Coordinates
    r_3   = sp.Matrix([     -.1939,   0,       0,  1 ]) # Frame 3 Coordinates
    r_4   = sp.Matrix([          0,   0,       0,  1 ]) # Frame 4 Coordinates
    r_5   = sp.Matrix([          0,   0,       0,  1 ]) # Frame 5 Coordinaates
    r_6   = sp.Matrix([          0,   0,    -.02,  1 ]) # Frame 6 Coordinates

    r_com          = [r_1, r_2, r_3, r_4, r_5, r_6]
    p_com_sym      = [T_home_matrices[i] * r_com[i]  for i in range(6)]

    m = sp.Matrix([0.9, 1.6, 0.8, 0.4, 0.2, .1 ]) # kg
    g = sp.Matrix([0, 0, -9.81, 0, 0, 0]) # Gravity Vector, m/s^2
    F = [m[i]*g for i in range(6)]


    # Z vectors of each link's CoM
    Z0        = sp.Matrix([0, 0, 1])
    Z_vectors_CoM = [T[:3, 2] for T in T_home_matrices]

    # Compute Jacobian Using the First Method leveraging each joint velocity is function of r*theta_dot, where r can be 
    # calculated from the cross product of Z and L (O_n - O_i-1)
    J_com = []
    for i in range(6):
        Z_i_1   = Z_vectors_CoM[i]
        o_i_1   = p_com_sym[i][0:3, 0]
        L       = p_com_sym[5][0:3, 0] - o_i_1
        J_i_0   = Z_i_1.cross(L)
        J_i     = J_i_0.col_join(Z_i_1)
        J_com.append(J_i)
    J_com  = sp.Matrix.hstack(*J_com)
    return J_com, p_com_sym, F

def get_joint_torques(J_CoM, F):
    F_n   = 5 # Newtons
    J_i, G = [], []
    for i in range(6):
        J_i = J_CoM[:, i]
        G_i = J_i.T * F[i]
        G.append(G_i)

    G     = sp.Matrix(G)                      # 6x1 force vector
    F_ext = sp.Matrix([0, 0, -F_n, 0, 0, 0])  # 6x1 force vector

    tau = J_CoM.T * F_ext

    joint_torques = G + tau

    print("Symbolic G(q)")
    sp.pprint(G)
    
    return joint_torques


def plot_workspace(T06_func):
    ''' This function computes the workspace of the robot given the symbolic transformation matrix T06'''
    # Define joint ranges
    joint_range = np.linspace(-np.pi/2,     np.pi/2, 4)  # Adjust range and resolution as needed
    origin = np.array([0, 0, 0, 1])

    # Compute workspace points
    workspace_points = []
    for q1 in joint_range:
        for q2 in joint_range:
            for q3 in joint_range:
                for q4 in joint_range:
                    for q5 in joint_range:
                        for q6 in joint_range:
                            joint_values = [q1, q2, q3, q4, q5, q6]
                            q_sym            = get_thetas(get_sympy_thetas(*joint_values))
                            T06_numeric_flat = T06_func(*q_sym)
                            T06_numeric      = np.array(T06_numeric_flat).reshape(4, 4)
                            p_curr           = T06_numeric @ origin
                            workspace_points.append([float(coord) for coord in p_curr[:3]])


    workspace_points = np.array(workspace_points)
    # Extract X, Y, Z coordinates
    x, y, z = workspace_points[:, 0], workspace_points[:, 1], workspace_points[:, 2]

    # Create 3D Mesh plot
    fig = go.Figure()

    # Mesh for shaded workspace
    fig.add_trace(go.Mesh3d(
        x=x, y=y, z=z,
        color='blue',
        opacity=0.5,
        alphahull=5  # surface tightness
    ))

    # Scatter plot for discrete points (optional)
    fig.add_trace(go.Scatter3d(
        x=x, y=y, z=z,
        mode='markers',
        marker=dict(size=2, color='red', opacity=0.8),
        name='Workspace Points'
    ))

    # Customize layout
    fig.update_layout(
        scene=dict(
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z',
        ),
        title="Robot Workspace, Joint movement limited to +/- pi/2",
    )
    # Show interactive plot
    fig.show()


#  MAIN SETUP

# Get the symbolic transformation matrices and Jacobian for our DH Reference Frames
T_home_matrices, J  = get_robot_transformations()

# Get the symbolic transformation matrices and Jacobian for our DH Reference Frames
J_CoM, p_com_sym, F = get_CoM_parameters(T_home_matrices)

# Get Symbolic Joint Torques for each Link 
joint_torques       = get_joint_torques(J_CoM, F)

print("Symbolic Jacobian for DH Frames")
sp.pprint(J, use_unicode=True)

print("Symbolic Jacobian for CoM Frames")
sp.pprint(J_CoM, use_unicode=True)

print("Symbolic Joint Torques")
sp.pprint(joint_torques, use_unicode=True)

# Input all fixed values into our symbolic transformation Matrices, Than lambdify with only the theta variables as inputs
# This step vastly increases the speed of computation later on
origin  = sp.Matrix([0, 0, 0, 1])
theta1_start, theta2_start, theta3_start     =  0, 0, 0
theta_4_start, theta_5_start, theta_6_start  =  0, 0, 0

start_values, theta_variables       = get_sympy_inputs(theta_1=theta1_start, theta_2=theta2_start, theta_3=theta3_start, theta_4=theta_4_start, theta_5=theta_5_start, theta_6=theta_6_start)
keys_to_remove                      = theta_variables
start_values_fixed                  = {key: start_values[key] for key in start_values if key not in keys_to_remove}
theta_current                       = get_sympy_thetas(theta_1=theta1_start, theta_2=theta2_start, theta_3=theta3_start, theta_4=theta_4_start, theta_5=theta_5_start, theta_6=theta_6_start)

func_mats, subs_mats         = get_lamdified(J, J_CoM, joint_torques, T_home_matrices, start_values_fixed, theta_variables)
J_func, T06_func, J_CoM_func = func_mats['J_func'], func_mats['T06_func'], func_mats['J_CoM_func']

T01_subs   = subs_mats['T01_subs']
T02_subs   = subs_mats['T02_subs']
T03_subs   = subs_mats['T03_subs']
T04_subs   = subs_mats['T04_subs']
T05_subs   = subs_mats['T05_subs']
T06_subs   = subs_mats['T06_subs']
J_CoM_subs = subs_mats['J_CoM_subs']    

torque_subs = subs_mats['joint_torques_subs']
torque_func = func_mats['joint_torques_func']

T_home_matrices_subs = [T01_subs, T02_subs, T03_subs, T04_subs, T05_subs, T06_subs]

#  Run Test Cases and Plot Home vs. Test Case
plot_test_cases(T_home_matrices_subs)

#  Plot Workspace
plot_workspace(T06_func)

# %%
# Define joint ranges


# %% Run Custom Case (Starting Position for Trajectory)
# Define joint ranges
joint_ranges = [np.linspace(-180, 180, 10) for _ in range(6)]  # Adjust range and resolution as needed

theta_1_start, theta_2_start, theta_3_start = 0, -np.pi/2, 0,
theta_4_start, theta_5_start, theta_6_start = 0,        0, 0 

case_title = "Starting Position (Case) to Draw Trajectory"   
home_origins, case_origins = run_test_case(
    theta_1=theta_1_start, 
    theta_2=theta_2_start, 
    theta_3=theta_3_start, 
    theta_4=theta_4_start, 
    theta_5=theta_5_start, 
    theta_6=theta_6_start, 
    T_home_matrices=T_home_matrices,
    case_title=case_title, 
    print_origins=False)


def get_desired_trajectory(phase_in=0, p_start=np.array([0,0,0,1]), phase_elapsed_time=0, 
                            stopped=False, omega=.1, linear_phase_velocity=.1, break_time=2,
                            r=.05, a=.05, b=.1):
    """ 
    Trajectory Planning, Draw Half Crlce + 3/4 of Rectangle below Half-Circle on the X-Z Plane
    Parameters: phase_in:              Current phase of trajectory
                p_start:               Starting position of Robot
                phase_elapsed_time:    Time elapsed in the current phase
                stopped:               Boolean indicating if the robot is stopped
                omega:                 Angular velocity of the semi-circle
                linear_phase_velocity: Linear velocity of the robot
                r:                     Radius of the semi-circle
                a:                     Distance between S and A
                b:                     Distance between A and B

    Returns:    p_out:                  Desired position of the robot
                v_out:                  Desired velocity of the robot
                w_out:                  Desired angular velocity of the robot
    """
    # Trajectory Points

    S  = [p_start[0],   p_start[1],   p_start[2]]
    S2 = [  S[0]-b,    S[1],     S[2] ]
    A  = [ S2[0],     S2[1],   S[2]-a ]
    B  = [  A[0]+b,   A[1],     A[2] ]

    # Phase 0: Move to the initial position
    if phase_in == 0:
        x_desired, y_desired, z_desired                = S[0], S[1], S[2]
        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        x_desired_dot, y_desired_dot, z_desired_dot    = 10.0, 0.0, 10

    #Phase 1: Move in a Semi-Circle
    elif phase_in == 1:
        theta = omega*phase_elapsed_time
       
        y_desired      = S[1]
        x_c, z_c       = S[0]-r, S[2]
        
        x_desired      = x_c + r * np.cos(theta)
        z_desired      = z_c + r * np.sin(theta)
        
        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        y_desired_dot      = 0
        x_desired_dot      =  -r * omega * np.sin(theta)
        z_desired_dot      =   r * omega * np.cos(theta)

    elif (phase==2) & stopped:
        x_desired, y_desired, z_desired                = S2[0], S2[1], S2[2]
        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        x_desired_dot, y_desired_dot, z_desired_dot    = 0.0, 0.0, 0.0

    # Phase 2: Move to Point A
    elif phase_in == 2:
        x_desired     = A[0]
        y_desired     = A[1]
        if (phase_elapsed_time-break_time)*linear_phase_velocity >= a:
            z_desired = A[2]
        else:
            z_desired = A[2]+a - (phase_elapsed_time-break_time)*linear_phase_velocity # Move desired target at speed of robot

        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        x_desired_dot, y_desired_dot, z_desired_dot    = 0, 0, -linear_phase_velocity

    elif (phase==3) & stopped:
        x_desired, y_desired, z_desired                = A[0], A[1], A[2]
        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        x_desired_dot, y_desired_dot, z_desired_dot    = 0.0, 0.0, 0.0

    # Phase 3: Move to Point B
    elif phase_in == 3:
        y_desired     = B[1]
        if (phase_elapsed_time-break_time)*linear_phase_velocity >= b:
            x_desired = B[0] # Phase Target
        else:
            x_desired = A[0] + (phase_elapsed_time-break_time)*linear_phase_velocity # Move to desired target at speed of robot

        z_desired     = B[2]
        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        x_desired_dot, y_desired_dot, z_desired_dot = linear_phase_velocity, 0, 0

    elif (phase==4) & stopped:
        x_desired, y_desired, z_desired                = B[0], B[1], B[2]
        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        x_desired_dot, y_desired_dot, z_desired_dot    = 0.0, 0.0, 0.0
    
    # Phase 4: Move Back to the initial position
    elif phase_in == 4:
        x_desired     = S[0]
        y_desired     = S[1]
        if (phase_elapsed_time-break_time)*linear_phase_velocity >= a:
            z_desired = S[2] # Phase Target
        else:
            z_desired = B[2] + (phase_elapsed_time-break_time)*linear_phase_velocity # Move desired target at speed of robot

        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        x_desired_dot, y_desired_dot, z_desired_dot = 0, 0, linear_phase_velocity
        

    else:
        x_desired, y_desired, z_desired                = S[0], S[1], S[2]
        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        x_desired_dot, y_desired_dot, z_desired_dot    = 0.0, 0.0, 0.0


    p_out = [x_desired, y_desired, z_desired]
    w_out = [wx_desired_dot, wy_desired_dot, wz_desired_dot]
    v_out = [x_desired_dot, y_desired_dot, z_desired_dot]

    p_out, w_out, v_out = np.array(p_out, dtype='float'), np.array(w_out, dtype='float'), np.array(v_out, dtype='float')
    return p_out, v_out, w_out


# %% Robot Control and Trajectory

# Constants and initial parameters, Angles in Radians
origin     = np.array([0, 0, 0, 1])
T          = 200              # Total time duration (s)
num_points = 1000             # Number of points along the trajectory
dt         = T / num_points   # Time step

# Trajectory Variables
r                     = 50/1000  # m
a                     = 50/1000  # m
b                     = 100/1000 # m
omega                 = 2 * np.pi / 150  # Angular speed for semi-circle trajectory
linear_phase_velocity = 25 / 10000       # m/s
break_time            = 10                # seconds

# PID Gains
k_p = 0.03   # Proportional gain
k_i = 0.02   # Integral gain
k_d = 0.01   # Derivative gain

# Initialize variables
theta_current = get_sympy_thetas(
    theta_1=theta_1_start,
    theta_2=theta_2_start,
    theta_3=theta_3_start,
    theta_4=theta_4_start,
    theta_5=theta_5_start,
    theta_6=theta_6_start
)
theta_numeric_values = get_thetas(theta_current)
T06_numeric_flat     = T06_func(*theta_numeric_values)
T06_numeric          = np.array(T06_numeric_flat).reshape(4, 4)
p_start              = T06_numeric @ origin

q           = np.array([theta_1_start, theta_2_start, theta_3_start, theta_4_start, theta_5_start, theta_6_start], dtype=float)
v_eff       = np.zeros(6)
p_curr_list = []
q_list      = []
q_dot_list  = []
error_list  = []
t_plot      = []

# Error variables
coord_error_int  = np.zeros(3)
coord_error_prev = np.zeros(3)

# Time and phase variables
curr_t              = 0
phase               = 1
t_start_phase       = 0
phase_elapsed_time  = 0
semi_circle_delta_t = np.pi / omega # Time required to complete 180 degrees of circle based on omega



joint_torques = []

def update_joint_positions(q, v_eff, dt):
    # Calculate joint velocities and update joint positions
    try:
        q_sym          = get_thetas(get_sympy_thetas(*q))
        J_numeric_flat = J_func(*q_sym)
        J_num          = np.array(J_numeric_flat).reshape(6, 6)
        q_dot          = np.linalg.pinv(J_num) @ v_eff

    except np.linalg.LinAlgError:
        print(f"Singularity at time {curr_t:.2f}s")
        q_dot = np.zeros(6)

    # Limit joint velocities to +/- .1 rad/s
    q_dot = np.clip(q_dot, -0.1, 0.1)

    # Update joint positions
    delta_q = q_dot * dt
    q_new   = q + delta_q
    q_new   = np.array([reduce_angle(angle) for angle in q_new])

    return q_new, q_dot

def control_loop(phase, q, curr_t, t_start_phase, coord_error_int, coord_error_prev):
    stopped = False
    phase_duration = {
        1: semi_circle_delta_t,
        2: break_time + (a / linear_phase_velocity),
        3: break_time + (b / linear_phase_velocity),
        4: break_time + (a / linear_phase_velocity)
    }

    while True:
        phase_elapsed_time = curr_t - t_start_phase

        # Check if phase is complete
        if phase_elapsed_time > phase_duration.get(phase, 0):
            return q, curr_t, coord_error_int, coord_error_prev, True  # Phase complete

        # Update stopped status based on phase and elapsed time
        if phase in [2, 3, 4] and phase_elapsed_time <= break_time:
            stopped = True
        else:
            stopped = False

        # Get desired trajectory
        p_desired, v_desired, w_desired = get_desired_trajectory(
            phase_in=phase,
            p_start=p_start,
            phase_elapsed_time=phase_elapsed_time,
            stopped=stopped,
            omega=omega,
            linear_phase_velocity=linear_phase_velocity,
            break_time=break_time,
            r=r,
            a=a,
            b=b
        )

        # Get current position
        q_sym            = get_thetas(get_sympy_thetas(*q))
        T06_numeric_flat = T06_func(*q_sym)
        T06_numeric      = np.array(T06_numeric_flat).reshape(4, 4)
        p_curr           = T06_numeric @ origin

        # Get Current Torques
        torques = torque_func(*q_sym)
        joint_torques.append(torques)

        # Compute errors
        coord_error       = p_desired - p_curr[:3]
        coord_error_int  += coord_error * dt
        coord_error_deriv = (coord_error - coord_error_prev) / dt
        coord_error_prev  = coord_error
        

        # Control signal
        error_norm = np.linalg.norm(coord_error)
        if error_norm < 1e-3:
            control_signal = np.zeros(3)
        else:
            control_signal = k_p * coord_error + k_i * coord_error_int + k_d * coord_error_deriv

        # Effective velocities
        v_l_eff = v_desired + control_signal
        v_w_eff = np.zeros(3)
        v_eff   = np.concatenate((v_l_eff, v_w_eff))

        # Update joint positions
        q, q_dot = update_joint_positions(q, v_eff, dt)

        # Record data
        p_curr_list.append(p_curr)
        q_list.append(q)
        q_dot_list.append(q_dot)
        error_list.append(coord_error)
        t_plot.append(curr_t)

        # Update time
        curr_t += dt
        
    return q, curr_t, coord_error_int, coord_error_prev, False  

# Main Trajectory PLanning and Control loop
while phase <= 4:
    print(f"Starting Phase {phase}")
    q, curr_t, coord_error_int, coord_error_prev, phase_complete = control_loop(
        phase, q, curr_t, t_start_phase, coord_error_int, coord_error_prev
    )
    if phase_complete:
        phase            += 1
        t_start_phase     = curr_t
        coord_error_int   = np.zeros(3)
        coord_error_prev  = np.zeros(3)

plot_joints(t_plot, q_list, p_curr_list, q_dot_list, error_list, joint_torques)

