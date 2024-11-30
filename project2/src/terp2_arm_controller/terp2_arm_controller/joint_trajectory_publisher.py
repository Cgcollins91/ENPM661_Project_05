import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import SetBool
import numpy as np
import sympy as sp

def get_symbolic_DH_matrix(i):
    """ Get the symbolic DH matrix for the ith joint of the UR5 robot"""
    theta = sp.symbols(f'theta_{i}')

    if i == 1 or i == 2:
        theta = theta - sp.pi / 2  # Explicitly subtract pi/2 for specific joints
    elif i == 3:
        theta = theta + sp.pi  # Explicitly subtract pi/2 for specific joints
    elif i == 4:
        theta = theta + sp.pi / 2

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

    a     = [       0,       0.85,   0.75,        0,          0,        0 ]
    alpha = [ -np.pi/2,     np.pi,      0,  -np.pi/2,  np.pi/2,         0 ]
    d     = [     0.05,      0.05,     0.1,     0.05,      0.2,      0.15 ]
    theta = [  theta_1,   theta_2, theta_3,   theta_4,  theta_5,  theta_6 ]
 

    for index, i in zip(indexes, range(1, 7)):
        values[sp.symbols(f'theta_{index}')] = theta[i-1]
        theta_variables.append(sp.symbols(f'theta_{index}'))
        values[sp.symbols(f'alpha_{index}')] = alpha[i-1]
        values[sp.symbols(f'a_{index}')]     = a[i-1]
        values[sp.symbols(f'd_{index}')]     = d[i-1]

    theta_variables = [theta for theta in theta_variables if isinstance(theta, sp.Basic)]

    return values, theta_variables

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

def get_CoM_parameters(T_home_matrices):
    ''' This function computes the symbolic transformation and Jacobian matrices for our Center of Masses'''
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

def get_thetas(theta_current):
    theta_1  = theta_current[sp.symbols('theta_1')]

    theta_2  = theta_current[sp.symbols('theta_2')]

    theta_3  = theta_current[sp.symbols('theta_3')]

    theta_4  = theta_current[sp.symbols('theta_4')]
    theta_5  = theta_current[sp.symbols('theta_5')]
    theta_6  = theta_current[sp.symbols('theta_6')]

    return [theta_1,  theta_2,  theta_3, theta_4, theta_5, theta_6]


class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',  # Controller's command topic
            10
        )

        self.target = [-1.4315, 0.045753, 0.823987]      
        T_home_matrices, J  = get_robot_transformations()
        # Get the symbolic transformation matrices and Jacobian for our DH Reference Frames
        # J_CoM, p_com_sym, F = get_CoM_parameters(T_home_matrices)

        # Get Symbolic Joint Torques for each Link 
        # joint_torques       = get_joint_torques(J_CoM, F)
        

        origin  = sp.Matrix([0, 0, 0, 1])
        theta1_start, theta2_start, theta3_start     =  0, 0, 0
        theta_4_start, theta_5_start, theta_6_start  =  0, 0, 0

        start_values, theta_variables       = get_sympy_inputs(theta_1=theta1_start, theta_2=theta2_start, theta_3=theta3_start, theta_4=theta_4_start, theta_5=theta_5_start, theta_6=theta_6_start)
        keys_to_remove                      = theta_variables
        start_values_fixed                  = {key: start_values[key] for key in start_values if key not in keys_to_remove}
        theta_current                       = get_sympy_thetas(theta_1=theta1_start, theta_2=theta2_start, theta_3=theta3_start, theta_4=theta_4_start, theta_5=theta_5_start, theta_6=theta_6_start)

        func_mats, subs_mats = get_lamdified(J, J_CoM, joint_torques, T_home_matrices, start_values_fixed, theta_variables)
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

        self.T_matrices = [T01_subs, T02_subs, T03_subs, T04_subs, T05_subs, T06_subs]
        self.T06_func = T06_func
        self.J_func = J_func
        self.timer_period = 0.1  # Publish every 0.01 seconds (100 Hz)
        self.timer = self.create_timer(self.timer_period, self.update_joint_positions)


        self.target_service = self.create_service(
            SetBool,
            'set_target_positions',
            self.set_target_positions
        )

        # Joint names (ensure these match your robot's joints)
        self.joint_names = [
            'joint_arm_1',
            'joint_arm_2',
            'joint_arm_3',
            'joint_arm_4',
            'joint_arm_5'
        ]

        self.gripper_names = [
            'joint_gripper_base',
            'joint_gripper_gear',
            'joint_gripper_pad1',
            'joint_gripper_pad2'
        ]

        self.start_time = self.get_clock().now()

        # Initialize joint positions and velocities
        self.q     = np.zeros(6)  # Joint positions
        self.q_dot = np.zeros(6)  # Joint velocities
        self.dt    = self.timer_period  # Time step

        # PID controller variables
        self.coord_error_int = np.zeros(3)
        self.coord_error_prev = np.zeros(3)
        self.moving = True  # Flag to indicate if joints are moving

    
    def update_joint_positions(self):
        """Update current joint positions towards target positions."""
        moving = False
        origin     = np.array([0, 0, 0, 1])
        T          = 20              # Total time duration (s)
        num_points = 1000             # Number of points along the trajectory
        dt         = T / num_points   # Time step
        theta_1_start, theta_2_start, theta_3_start     =  0, 0, 0
        theta_4_start, theta_5_start, theta_6_start     =  0, 0, 0

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
        T06_numeric_flat     = self.T06_func(*theta_numeric_values)
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
                


        for i, joint in enumerate(self.joint_names):
            step = self.joint_step[joint]
            current_position = self.current_joint_positions[i]
            target_position  = self.target_joint_positions[joint]

            if abs(current_position - target_position) > step:
                self.current_joint_positions[i] += step * np.sign(target_position - current_position)
                self.get_logger().info(f"Joint {joint} moving to {self.current_joint_positions[i]}")
    
                moving = True
            else:
                self.current_joint_positions[i] = target_position
        self.moving = moving

    def publish_joint_trajectory(self):
        """Publish the joint trajectory to the controller."""
         # Create JointTrajectory message
        time_to_move = 20 
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = self.q[:5].tolist()  # First 5 joints
        point.time_from_start = Duration(seconds=time_to_move).to_msg()  # Time to reach the positions

        trajectory_msg.points.append(point)

        gripper_msg = JointTrajectory()
        gripper_msg.joint_names = ['joint_gripper_base']  # Only the 6th joint

        # Create a trajectory point for gripper joint
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [self.q[5]]  # 6th joint position
        gripper_point.time_from_start = Duration(seconds=time_to_move).to_msg()

        gripper_msg.points.append(gripper_point)

        # Publish the trajectory
        self.publisher.publish(trajectory_msg)
        self.get_logger().info(f"Publishing joint trajectory: {trajectory_msg}")


    def set_target_positions(self, request, response):
        """Service callback to update target joint positions."""
        # Example: Update all joints to a new target
        new_target = np.pi / 4 if request.data else 0.0
        for joint in self.joint_names:
            self.target_joint_positions[joint] = new_target
        response.success = True
        response.message = "Updated target positions"
        self.get_logger().info(f"Target positions updated to {new_target}")
        self.moving = True
        return response


def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_publisher = JointTrajectoryPublisher()
    rclpy.spin(joint_trajectory_publisher)

    # Cleanup
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
