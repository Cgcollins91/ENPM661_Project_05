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
    """ Get the symbolic DH matrix for the ith joint of the UR5 robot"""
    theta = sp.symbols(f'theta_{i}')
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
    
    values = {}
    theta_variables= []
    indexes = ['1',       '2',    '3',     '4',      '5',       '6']

    a     = [      0,       0,  737.31,  387.8,        0,        0 ]
    alpha = [      0, -np.pi/2,      0,      0,  np.pi/2, -np.pi/2 ]
    d     = [  183.3,        0,      0,  172.3,     95.5,    115.5 ]
    theta = [ theta_1, theta_2, theta_3, theta_4, theta_5,  theta_6 ]
 

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

    for index, i in zip(indexes, range(1, 10)):
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
    origins_padded                             = [origin, o1,  o2,  o3,  o4, o5, o6 ]

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

    theta_current, theta_variables  = get_sympy_inputs(theta_1=0, theta_2=0, theta_3=0, theta_4=0, theta_5=0, theta_6=0)
    home_origins, origins_padded, T_matrices = full_forward_kinematics_numeric(T_home_matrices, theta_current)
    
    

    theta_current, theta_variables  = get_sympy_inputs(theta_1=theta_1, theta_2=theta_2, theta_3=theta_3, theta_4=theta_4, theta_5=theta_5, theta_6=theta_6)
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
            text=f'Home O {i+1}',
            name=f'Home O {i+1}'
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
                name=f'Home O {i+1}',
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
            text=f'Case O {i+1}',
            name=f'Case O {i+1}'
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


    if print_origins:
        print(case_title)
        for i, o in enumerate([o1, o2, o3, o4, o5, o6], start=1):
            print(f"\nOrigin {i}:")
            sp.pprint(o, use_unicode=True)
   
    
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


def get_desired_trajectory(phase_in=0, p_start=np.array([0,0,0,1]), phase_elapsed_time=0, 
                            stopped=False, omega=.1, linear_phase_velocity=.1, break_time=2):
    """ 
    Trajectory Planning
    Parameters: phase_in:              Current phase of trajectory
                p_start:               Starting position of Robot
                phase_elapsed_time:    Time elapsed in the current phase
                stopped:               Boolean indicating if the robot is stopped
                omega:                 Angular velocity of the semi-circle
                linear_phase_velocity: Linear velocity of the robot

    Returns:    p_out:                  Desired position of the robot
                v_out:                  Desired velocity of the robot
                w_out:                  Desired angular velocity of the robot
    """
    # Trajectory Points
    S  = [p_start[0],   p_start[1],   p_start[2]]
    S2 = [  S[0],   S[1]-10,     S[2]]
    A  = [ S2[0],     S2[1], S[2]-5]
    B  = [  A[0],   A[1]+10,     A[2]]

    # Phase 0: Move to the initial position
    if phase_in == 0:
        x_desired, y_desired, z_desired                = S[0], S[1], S[2]
        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        x_desired_dot, y_desired_dot, z_desired_dot    = 0.0, 10, 10

    #Phase 1: Move in a Semi-Circle
    elif phase_in == 1:
        theta = omega*phase_elapsed_time
        r              = 5               # Radius of the circle
        x_desired      = S[0]
        y_c, z_c       = S[1]-5, S[2]
        
        y_desired      = y_c - r * np.cos(theta)
        z_desired      = z_c - r * np.sin(theta)
        
        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        x_desired_dot      = 0
        y_desired_dot      = -r * omega * np.sin(theta)
        z_desired_dot      =  r * omega * np.cos(theta)

    elif (phase==2) & stopped:
        x_desired, y_desired, z_desired                = S2[0], S2[1], S2[2]
        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        x_desired_dot, y_desired_dot, z_desired_dot    = 0.0, 0.0, 0.0

    # Phase 2: Move to Point A
    elif phase_in == 2:
        x_desired     = A[0]
        y_desired     = A[1]
        if (phase_elapsed_time-break_time)*linear_phase_velocity >= 5:
            z_desired = A[2]
        else:
            z_desired = A[2]+5 - (phase_elapsed_time-break_time)*linear_phase_velocity # Move desired target at speed of robot

        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        x_desired_dot, y_desired_dot, z_desired_dot    = 0, 0, -linear_phase_velocity

    elif (phase==3) & stopped:
        x_desired, y_desired, z_desired                = A[0], A[1], A[2]
        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        x_desired_dot, y_desired_dot, z_desired_dot    = 0.0, 0.0, 0.0

    # Phase 3: Move to Point B
    elif phase_in == 3:
        x_desired     = B[0]
        if (phase_elapsed_time-break_time) >= 10:
            y_desired = B[1] # Phase Target
        else:
            y_desired = A[1] + (phase_elapsed_time-break_time)*linear_phase_velocity # Move to desired target at speed of robot

        z_desired     = B[2]
        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        x_desired_dot, y_desired_dot, z_desired_dot = 0, linear_phase_velocity, 0

    elif (phase==4) & stopped:
        x_desired, y_desired, z_desired                = B[0], B[1], B[2]
        wx_desired_dot, wy_desired_dot, wz_desired_dot = 0.0, 0.0, 0.0
        x_desired_dot, y_desired_dot, z_desired_dot    = 0.0, 0.0, 0.0
    
    # Phase 4: Move Back to the initial position
    elif phase_in == 4:
        x_desired     = S[0]
        y_desired     = S[1]
        if (phase_elapsed_time-break_time)*linear_phase_velocity >= 5:
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


def plot_joints(t, q_list, p_curr_list, q_dot_list, error_list):
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

    

#  Get Synbolic Transformation Matrices for each joint
T01  = get_symbolic_DH_matrix('1')
T2   = get_symbolic_DH_matrix('2')
T3   = get_symbolic_DH_matrix('3')
T4   = get_symbolic_DH_matrix('4')
T5   = get_symbolic_DH_matrix('5')
T6   = get_symbolic_DH_matrix('6')

# Compute the symbolic transformation matrices for each joint relative to the base frame
T02  = T01*T2
T03  = T02*T3
T04  = T03*T4
T05  = T04*T5
T06  = T05*T6

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

sp.pprint(J, use_unicode=True)


# Input all fixed values into our symbolic transformation Matrices, Than lambdify the Jacobian with only the theta variables as inputs
# This step vastly increases the speed of computation later on
origin  = sp.Matrix([0, 0, 0, 1])
theta1_start, theta2_start, theta3_start, theta_4_start, theta_5_start, theta_6_start  = np.pi/2, 0, np.pi/2, 0, 0, 0

start_values, theta_variables       = get_sympy_inputs(theta_1=theta1_start, theta_2=theta2_start, theta_3=theta3_start, theta_4=theta_4_start, theta_5=theta_5_start, theta_6=theta_6_start)
keys_to_remove                      = theta_variables
start_values_fixed                  = {key: start_values[key] for key in start_values if key not in keys_to_remove}
theta_current                       = get_sympy_thetas(theta_1=theta1_start, theta_2=theta2_start, theta_3=theta3_start, theta_4=theta_4_start, theta_5=theta_5_start, theta_6=theta_6_start)

J_subs   = J.subs(start_values_fixed)
T06_subs = T06.subs(start_values_fixed)

# Flatten Jacobian for lambdification
J_flat = J_subs.reshape(J_subs.shape[0]*J_subs.shape[1], 1)
J_func = sp.lambdify(theta_variables, J_flat, modules='numpy')

# Flatten T06 for lambdification
T06_flat = T06_subs.reshape(T06_subs.shape[0]*T06_subs.shape[1], 1)
T06_func = sp.lambdify(theta_variables, T06_flat, modules='numpy')


# %% Run Test Cases and Plot Home vs. Test Case
plot_test_cases(T_home_matrices)


# %% Run Custom Case
theta_1, theta_2, theta_3, theta_4,  theta_5, theta_6 = np.pi/2, 0, 0, np.pi/2, 0, 0
case_title = "Case 5, theta_1 = pi/2, theta_4 = pi/2"   
home_origins, case_origins = run_test_case(theta_1=theta_1, theta_2=theta_2, theta_3=theta_3, theta_4=theta_4, theta_5=theta_5, theta_6=theta_6, T_home_matrices=T_home_matrices, case_title=case_title, print_origins=False)

# %% Robot Control and Trajectory

# Constants and initial parameters, Angles in Radians
origin = np.array([0, 0, 0, 1])
theta1_start, theta2_start, theta3_start, theta_4_start, theta_5_start, theta_6_start = 0, np.pi/2, -np.pi/2, np.pi/2, 0, 0
T          = 25               # Total time duration
num_points = 1000             # Number of points along the trajectory
dt         = T / num_points   # Time step

omega                 = 2 * np.pi / 8  # Angular speed for semi-circle trajectory
linear_phase_velocity = 2.5            # cm/s
break_time            = 1              # seconds

# PID Gains
k_p = 0.01   # Proportional gain
k_i = 0.01   # Integral gain
k_d = 0.01   # Derivative gain

# Initialize variables
theta_current = get_sympy_thetas(
    theta_1=theta1_start,
    theta_2=theta2_start,
    theta_3=theta3_start,
    theta_4=theta_4_start,
    theta_5=theta_5_start,
    theta_6=theta_6_start
)
theta_numeric_values = get_thetas(theta_current)
T06_numeric_flat     = T06_func(*theta_numeric_values)
T06_numeric          = np.array(T06_numeric_flat).reshape(4, 4)
p_start              = T06_numeric @ origin

q           = np.array([theta1_start, theta2_start, theta3_start, theta_4_start, theta_5_start, theta_6_start], dtype=float)
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
        2: break_time + (5 / linear_phase_velocity),
        3: break_time + (10 / linear_phase_velocity),
        4: break_time + (5 / linear_phase_velocity)
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
            break_time=break_time
        )

        # Get current position
        q_sym            = get_thetas(get_sympy_thetas(*q))
        T06_numeric_flat = T06_func(*q_sym)
        T06_numeric      = np.array(T06_numeric_flat).reshape(4, 4)
        p_curr           = T06_numeric @ origin

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

plot_joints(t_plot, q_list, p_curr_list, q_dot_list, error_list)
