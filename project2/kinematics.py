# File: problem1.py
# Description:
# Author: James Fehrmann
# Created: 2024-11
# License: MIT License
# Version: 1.0.0

import math

import sympy as smp
import numpy as np
from collections import namedtuple
import matplotlib.pyplot as plotter
from tabulate import tabulate

dh_entry = namedtuple('dh_entry', ['theta', 'alpha', 'd', 'r'])
pose = namedtuple('pose', ['x', 'y', 'z', 'R_x', 'R_y', 'R_z'])


def identity_matrix(size):
    i = []
    for r in range(1, size + 1):
        row = []
        for c in range(1, size + 1):
            if r == c:
                row.append(1)
            else:
                row.append(0)
        i.append(row)
    return smp.Matrix(i)


def round_matrix(this_matrix):
    return this_matrix.copy().applyfunc(lambda x: smp.N(x, 4))


def create_parametric_dh_matrix(joint_id):
    j = str(joint_id)
    theta = smp.symbols('θ' + j)
    alpha = smp.symbols('α' + j)
    d = smp.symbols('d' + j)
    r = smp.symbols('r' + j)

    r11 = smp.cos(theta)
    r12 = -smp.sin(theta) * smp.cos(alpha)
    r13 = smp.sin(theta) * smp.sin(alpha)
    r14 = r * smp.cos(theta)
    r21 = smp.sin(theta)
    r22 = smp.cos(theta) * smp.cos(alpha)
    r23 = -smp.cos(theta) * smp.sin(alpha)
    r24 = r * smp.sin(theta)
    r31 = 0
    r32 = smp.sin(alpha)
    r33 = smp.cos(alpha)
    r34 = d
    transform_matrix = [
        [r11, r12, r13, r14],
        [r21, r22, r23, r24],
        [r31, r32, r33, r34],
        [0, 0, 0, 1]
    ]
    return smp.Matrix(transform_matrix)


def theta_subber2(matrix, dh_table, joint_values):
    thetas = []
    for i in range(len(dh_table)):
        thetas.append(smp.symbols('θ' + str(i + 1)))
    lambda_matrix = smp.lambdify(thetas, matrix, modules="numpy")
    values = []
    for dh, joint in zip(dh_table, joint_values):
        values.append(float(dh.theta + joint))
    return smp.Matrix(lambda_matrix(*values))


def theta_subber(matrix, dh_table, joint_values):
    new_matrix = matrix.copy()
    sub_pairs = []
    for i in range(len(dh_table)):
        theta = smp.symbols('θ' + str(i + 1))
        value = dh_table[i].theta + joint_values[i]
        sub_pairs.append((theta, value))
    return new_matrix.subs(sub_pairs)


def pre_eval_dh_transforms(transform_matrix, dh_table):
    new_transform_matrix = transform_matrix.copy()
    i = 1
    for entry in dh_table:
        j = str(i)
        alpha = smp.symbols('α' + j)
        d = smp.symbols('d' + j)
        r = smp.symbols('r' + j)
        new_transform_matrix = new_transform_matrix.subs(alpha, entry.alpha)
        new_transform_matrix = new_transform_matrix.subs(d, entry.d)
        new_transform_matrix = new_transform_matrix.subs(r, entry.r)
        i = i + 1
    return new_transform_matrix


def jacobian_linear_functions(q_function_vector, link_count):
    x_row = []
    y_row = []
    z_row = []
    for i in range(link_count):
        theta = 'θ' + str(i + 1)
        x_row.append(smp.diff(q_function_vector[0], theta))
        y_row.append(smp.diff(q_function_vector[1], theta))
        z_row.append(smp.diff(q_function_vector[2], theta))
    return smp.Matrix([x_row, y_row, z_row])


def jacobian_angular_values(transforms, dh_table, joint_values):
    z_matrix = []
    for i in range(len(joint_values)):
        transform_evaluated = theta_subber(transforms[i + 1], dh_table, joint_values)
        z1 = transform_evaluated[0, 2]  # [1 - 1, 3 - 1]
        z2 = transform_evaluated[1, 2]  # [2 - 1, 3 - 1]
        z3 = transform_evaluated[2, 2]  # [3 - 1, 3 - 1]
        z_matrix.append([z1, z2, z3])
    jw = smp.Matrix(z_matrix).transpose()
    return jw


def jacobian_evaluated(jl_function, dh_table, joint_values, transforms):
    jl_val = theta_subber2(jl_function, dh_table, joint_values)
    jw_val = jacobian_angular_values(transforms, dh_table, joint_values)
    j_combined = smp.Matrix.vstack(jl_val, jw_val)
    return j_combined


def jacobian_inverse_dls(jacobian, lambda_factor):
    identity = smp.eye(jacobian.shape[1])
    regularization = lambda_factor ** 2 * identity
    damped_inverse = (jacobian.T * jacobian + regularization).inv() * jacobian.T
    return damped_inverse


def compute_theta_dots(j_inv, q_dot):
    theta_dots = j_inv * q_dot
    return theta_dots


def increment_thetas(joint_values, theta_dots, dt):
    for i in range(len(joint_values)):
        joint_values[i] += theta_dots[i] * dt
    return joint_values


def q_end_effector_circle_xy(circle_radius, draw_time):
    omega = 2 * smp.pi / draw_time
    phi = omega * smp.symbols('t')
    x = smp.symbols('x0') - circle_radius + circle_radius * smp.cos(phi)
    y = smp.symbols('y0') + circle_radius * smp.sin(phi)
    z = smp.symbols('z0')
    Rx = smp.symbols('i0')
    Ry = smp.symbols('j0')
    Rz = smp.symbols('k0')
    q_function_vector = smp.Matrix([x, y, z, Rx, Ry, Rz])
    return q_function_vector


def q_end_effector_circle_yz(circle_radius, draw_time):
    omega = 2 * smp.pi / draw_time
    phi = omega * smp.symbols('t')
    x = smp.symbols('x0')
    y = smp.symbols('y0') - circle_radius + circle_radius * smp.cos(phi)
    z = smp.symbols('z0') + circle_radius * smp.sin(phi)
    Rx = smp.symbols('i0')
    Ry = smp.symbols('j0')
    Rz = smp.symbols('k0')
    q_function_vector = smp.Matrix([x, y, z, Rx, Ry, Rz])
    return q_function_vector




def q_end_effector_line(dx, dy, dz, draw_time):
    x = smp.symbols('x0') + ((dx / draw_time) * smp.symbols('t'))
    y = smp.symbols('y0') + ((dy / draw_time) * smp.symbols('t'))
    z = smp.symbols('z0') + ((dz / draw_time) * smp.symbols('t'))
    Rx = smp.symbols('i0')
    Ry = smp.symbols('j0')
    Rz = smp.symbols('k0')
    q_function_vector = smp.Matrix([x, y, z, Rx, Ry, Rz])
    return q_function_vector


def q_end_effector_ramped_line(dx, dy, dz, draw_time):
    t = smp.symbols('t')
    x = smp.symbols('x0') + (dx / 2) + ((dx / 2) * smp.sin((smp.pi / draw_time) * t - (smp.pi / 2)))
    y = smp.symbols('y0') + (dy / 2) + ((dy / 2) * smp.sin((smp.pi / draw_time) * t - (smp.pi / 2)))
    z = smp.symbols('z0') + (dz / 2) + ((dz / 2) * smp.sin((smp.pi / draw_time) * t - (smp.pi / 2)))
    Rx = smp.symbols('i0')
    Ry = smp.symbols('j0')
    Rz = smp.symbols('k0')
    q_function_vector = smp.Matrix([x, y, z, Rx, Ry, Rz])
    return q_function_vector


def compute_q_end_effector(q_function_vector, offset, t):
    x = q_function_vector[0].subs({'x0': offset[0], 't': t})
    y = q_function_vector[1].subs({'y0': offset[1], 't': t})
    z = q_function_vector[2].subs({'z0': offset[2], 't': t})
    Rx = q_function_vector[3].subs({'i0': offset[3], 't': t})
    Ry = q_function_vector[4].subs({'j0': offset[4], 't': t})
    Rz = q_function_vector[5].subs({'k0': offset[5], 't': t})
    q_vector = smp.Matrix([x, y, z, Rx, Ry, Rz]).evalf()
    return q_vector


def q_dot_end_effector(q_function_vector):
    x = smp.diff(q_function_vector[0], 't')
    y = smp.diff(q_function_vector[1], 't')
    z = smp.diff(q_function_vector[2], 't')
    Rx = smp.diff(q_function_vector[3], 't')
    Ry = smp.diff(q_function_vector[4], 't')
    Rz = smp.diff(q_function_vector[5], 't')
    q_dot_function_vector = smp.Matrix([x, y, z, Rx, Ry, Rz])
    return q_dot_function_vector


def compute_q_dot_end_effector(q_dot_function_vector, t):
    x_dot = q_dot_function_vector[0].subs({'t': t}).evalf()
    y_dot = q_dot_function_vector[1].subs({'t': t}).evalf()
    z_dot = q_dot_function_vector[2].subs({'t': t}).evalf()
    Rx_dot = q_dot_function_vector[3].subs({'t': t}).evalf()
    Ry_dot = q_dot_function_vector[4].subs({'t': t}).evalf()
    Rz_dot = q_dot_function_vector[5].subs({'t': t}).evalf()
    q_dot_vector = smp.Matrix([x_dot, y_dot, z_dot, Rx_dot, Ry_dot, Rz_dot])
    return q_dot_vector


def plot3d(path_data, goal_data, draw_time, dt, joint_values_history, dh_table, elevation, azimuth):
    plot_x = []
    plot_y = []
    plot_z = []

    fig = plotter.figure(figsize=(20, 20), dpi=300)
    ax = fig.add_subplot(111, projection='3d')

    # plot path
    for p in path_data:
        plot_x.append(p[0])
        plot_y.append(p[1])
        plot_z.append(p[2])
    ax.plot(plot_x, plot_y, plot_z, color='b', linestyle='-', linewidth=1)

    link_count = len(joint_values_history[0])
    transforms = prebuild_transforms(link_count, dh_table)
    q_functions = []
    for i in range(link_count + 1):
        q_functions.append(forward_position_kinematics(transforms[i]))

    for i in range(0, len(joint_values_history), 20):
        arm_x = [0]
        arm_y = [0]
        arm_z = [0]
        for q_function in q_functions:
            blah = compute_forward_position_kinematics(q_function, dh_table, joint_values_history[i])
            arm_x.append(blah[0])
            arm_y.append(blah[1])
            arm_z.append(blah[2])
        ax.plot(arm_x, arm_y, arm_z, color='r', linestyle='-', linewidth=1)

    # plot goal
    for p in goal_data:
        plot_x.append(p[0])
        plot_y.append(p[1])
        plot_z.append(p[2])
    ax.plot(plot_x, plot_y, plot_z, color='g', linestyle=':', linewidth=1)

    # tidy the plot
    plotter.axis('equal')
    ax.view_init(elev=elevation, azim=azimuth)
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_zlabel('Z Coordinate')
    ax.set_title(f"Robot Trajectory (t_total={draw_time}s, dt={dt}s)")
    plotter.show()
    return


def plot(path_data, goal_data, draw_time, dt, plot_mode):
    plot_x = []
    plot_y = []
    plot_z = []

    # plot path
    for p in path_data:
        plot_x.append(p[0])
        plot_y.append(p[1])
        plot_z.append(p[2])
    if plot_mode == 'xy':
        plotter.plot(plot_x, plot_y, color='b', linestyle='-', linewidth=1, zorder=3)
    elif plot_mode == 'yz':
        plotter.plot(plot_y, plot_z, color='b', linestyle='-', linewidth=1, zorder=3)
    for i in range(1, len(plot_x)):
        dx = float(plot_x[i] - plot_x[i - 1])
        dy = float(plot_y[i] - plot_y[i - 1])
        dz = float(plot_z[i] - plot_z[i - 1])
        x = plot_x[i - 1]
        y = plot_y[i - 1]
        z = plot_z[i - 1]
        if plot_mode == 'xy':
            plotter.quiver(x + 0.1, y + 0.1, dx, dy, angles='xy', scale_units='xy', scale=2, zorder=4, color='r', width=0.004)
        elif plot_mode == 'yz':
            plotter.quiver(y + 0.1, z + 0.1, dy, dz, angles='xy', scale_units='xy', scale=2, zorder=4, color='r', width=0.004)
    plot_x.clear()
    plot_y.clear()
    plot_z.clear()

    # plot goal
    for p in goal_data:
        plot_x.append(p[0])
        plot_y.append(p[1])
        plot_z.append(p[2])
    if plot_mode == 'xy':
        plotter.plot(plot_x, plot_y, color='g', linestyle=':', linewidth=1, zorder=1)
    elif plot_mode == 'yz':
        plotter.plot(plot_y, plot_z, color='g', linestyle=':', linewidth=1, zorder=1)

    # tidy the plot
    plotter.axis('equal')
    if plot_mode == 'xy':
        plotter.xlabel('X Coordinate')
        plotter.ylabel('Y Coordinate')
    elif plot_mode == 'yz':
        plotter.xlabel('Y Coordinate')
        plotter.ylabel('Z Coordinate')
    plotter.title(f"Robot Trajectory (t_total={draw_time}s, dt={dt}s)")
    plotter.show()
    return


def plot_torque_over_time(torque_history, draw_time, dt, title):
    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
    plot_x = []
    plot_y = []

    link_count = len(torque_history[0])

    for i in range(link_count):
        time = 0
        for t in torque_history:
            torque = t[i]
            plot_x.append(time)
            plot_y.append(torque / 1000)
            time += dt
        plotter.plot(plot_x, plot_y, color=colors[i], linestyle='-', linewidth=1, zorder=3)
        plot_x.clear()
        plot_y.clear()

    # tidy the plot
    #plotter.axis('equal')
    plotter.xlabel('Elapsed Time (s)')
    plotter.ylabel('Total Torque (Nm)')
    plotter.title(f'{title} (t_total={draw_time}s, dt={dt}s)')
    plotter.show()
    return


def prebuild_transforms(link_count, dh_table):
    transforms = []
    transform = identity_matrix(4)
    transforms.append(transform.copy())
    for i in range(link_count):
        transform_i = create_parametric_dh_matrix(i + 1)
        transform = transform * transform_i
        transforms.append(pre_eval_dh_transforms(transform.copy(), dh_table))
    return transforms


def forward_position_kinematics(transform):
    x = transform[0, 3]  # [1 - 1, 4 - 1]
    y = transform[1, 3]  # [2 - 1, 4 - 1]
    z = transform[2, 3]  # [3 - 1, 4 - 1]
    Rx = smp.atan2(transform[2, 1], transform[2, 2])
    e1 = (transform[0, 0]) ** 2  # [1 - 1, 1 - 1]
    e2 = (transform[1, 0]) ** 2  # [2 - 1, 1 - 1]
    Ry = smp.atan2(-1 * transform[2, 0], smp.sqrt(e1 + e2))
    Rz = smp.atan2(transform[1, 0], transform[0, 0])
    q_function_vector = smp.Matrix([x, y, z, Rx, Ry, Rz])
    return q_function_vector


def compute_forward_position_kinematics(q_function_vector, dh_table, joint_values):
    q_vector = theta_subber2(q_function_vector, dh_table, joint_values)
    return q_vector


def shape_waypoints(robot_start, path_times, path_functions, model_defs):
    this_waypoint = robot_start.copy()
    waypoints = [this_waypoint.copy()]

    for i in range(len(path_functions)):
        next_waypoint = compute_q_end_effector(path_functions[i], this_waypoint, path_times[i])
        waypoints.append(next_waypoint.copy())
        this_waypoint = next_waypoint.copy()

    return waypoints


def shape_paths(model_defs, model_times, robot_start, draw_start, startup_time):
    x0 = draw_start[0] - robot_start[0]
    y0 = draw_start[1] - robot_start[1]
    z0 = draw_start[2] - robot_start[2]

    path_functions = []
    path_times = []
    if startup_time > 0 and (x0 != 0 or y0 != 0 or z0 != 0):
        path_functions.append(q_end_effector_ramped_line(x0, y0, z0, startup_time))
        path_times.append(startup_time)
    for i in range(len(model_defs)):
        if model_defs[i].shape == 'xy_circle':
            path_functions.append(q_end_effector_circle_xy(model_defs[i].size_x, 2 * model_times[i]))
        elif model_defs[i].shape == 'yz_circle':
            path_functions.append(q_end_effector_circle_yz(model_defs[i].size_y, 2 * model_times[i]))
        elif model_defs[i].shape == 'line':
            path_functions.append(
                q_end_effector_line(model_defs[i].size_x, model_defs[i].size_y, model_defs[i].size_z, model_times[i]))
        elif model_defs[i].shape == 'ramped_line':
            path_functions.append(
                q_end_effector_ramped_line(model_defs[i].size_x, model_defs[i].size_y, model_defs[i].size_z,
                                           model_times[i]))
        else:
            print('ERROR: Unknown Model Type!')
    path_times += model_times

    path_waypoints = shape_waypoints(robot_start, path_times, path_functions, model_defs)
    return path_functions, path_waypoints, path_times


path = namedtuple('path', ['shape', 'size_x', 'size_y', 'size_z'])


def model_definition0(shape_size):
    model_paths = [
    ]
    return model_paths


def model_definition1(shape_size):
    model_paths = [
        path("yz_circle", 0, shape_size, shape_size),
        path('line', 0, 0, -shape_size),
        path('ramped_line', 0, shape_size * 2, 0),
        path('ramped_line', 0, 0, shape_size),
    ]
    return model_paths


def model_definition2(shape_size):
    model_paths = [
        path("xy_circle", shape_size, shape_size, 0),
        path('line', 0, -shape_size, 0),
        path('ramped_line', shape_size * 2, 0, 0),
        path('ramped_line', 0, shape_size, 0),
    ]
    return model_paths

def empty_model(shape_size):
    return []


def model_path_lengths(model_paths):
    model_lengths = []
    for model_path in model_paths:
        if model_path.shape == "xy_circle":
            model_lengths.append(model_path.size_x * math.pi)
        elif model_path.shape == "yz_circle":
            model_lengths.append(model_path.size_y * math.pi)
        else:
            model_lengths.append(math.sqrt(model_path.size_x ** 2 + model_path.size_y ** 2 + model_path.size_z ** 2))
    return model_lengths


def model_path_times(model_lengths, draw_time):
    model_times = []
    ds = sum(model_lengths)
    for length in model_lengths:
        t = draw_time * (length / ds)
        model_times.append(t)
    return model_times


def path_picker(time, t):
    n = 0
    t_acc = 0
    while time > t_acc + t[n] and n < len(t) - 1:
        t_acc += t[n]
        n += 1
    draw_time_offset = time - t_acc
    return n, draw_time_offset


def compute_center_of_mass(dh_table):
    center_of_mass = []
    for entry in dh_table:
        com_x = entry.r / 2
        com_y = 0
        com_z = entry.d / 2
        center_of_mass.append([com_x, com_y, com_z])
    return center_of_mass


def potential_energy(link_masses, link_centers, transforms):
    gravity = smp.Matrix([0.0, 0.0, -9.81])
    p_function = 0
    for i in range(len(link_masses)):
        link_center = smp.Matrix(link_centers[i] + [1])
        print(f'Link{i + 1} Center: {link_center}')
        transform = transforms[i]  #transforms[i] acts like i-1 because the first transform is identity for world frame
        print(f'Transform for Link{i + 1}: {transform}')
        center_of_mass = transform * link_center
        print(f'Center of Mass for Link{i + 1}: {center_of_mass}')
        mass_scalar = link_masses[i]
        print(f'The mass of link{i+1}: {mass_scalar}kg')
        p = mass_scalar * gravity[2] * center_of_mass[2]
        print(p)
        p_function += p
    # p_function *= -1
    return p_function


def static_lagrangian(p_function, link_count):
    lagrangian = []
    for i in range(link_count):
        theta = 'θ' + str(i+1)
        del_p_del_q = smp.diff(p_function, theta)
        lagrangian.append(del_p_del_q)
    print(f'Gravity Matrix: {lagrangian}')
    return lagrangian


def compute_static_lagrangian(lagrangian, dh_table, joint_values):
    return theta_subber2(lagrangian, dh_table, joint_values)


def compute_joint_torques2(p_function, dh_table, joint_values):
    lagrangian = static_lagrangian(p_function, len(joint_values))
    torques = compute_static_lagrangian(lagrangian, dh_table, joint_values)
    return torques

def compute_external_force(end_effector_force, jj_val):
   return jj_val.T * smp.Matrix(end_effector_force)


def run_simulation(model_number, lambda_factor, use_scaled_lambda, plot_elevation, plot_azimuth, joint_values):
    verbose = True

    path_data = []
    goal_data = []

    # Scenario Constants
    draw_time = 200
    startup_time = 5
    circle_radius = 50
    time_delta = 2
    time_scale = 10
    dt = time_delta / time_scale

    rad90 = math.pi / 2
    rad45 = rad90 / 2
    rad180 = math.pi
    rad0 = 0.0

    # DH Table
    dh_table = [
        dh_entry(-rad90, -rad90,  50.0, 0.0),
        dh_entry(-rad90, rad180,  50.0, 850.0),
        dh_entry(rad180,   rad0, 100.0, 750.0),
        dh_entry(rad90,  -rad90,  50.0, 0.0),
        dh_entry(rad90,   rad90, 200.0, 0.0),
        dh_entry(rad0,     rad0,  50.0, 0.0)
    ]
    print(f'DH Table:')
    print(tabulate(dh_table, tablefmt="grid"))

    # Starting Joint Values and Transforms
    joint_values_history = [joint_values.copy()]
    transforms = prebuild_transforms(len(joint_values), dh_table)
    q_function = forward_position_kinematics(transforms[len(joint_values)])
    jl_function = jacobian_linear_functions(q_function, len(joint_values))
    print(f'Forward Position Kinematics: {q_function}')
    print(f'Jacobian Linear Functions: {jl_function}')

    # Dynamics
    link_masses = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    end_effector_force = [0.0, 0.0, -5.0, 0.0, 0.0, 0.0]
    link_centers = compute_center_of_mass(dh_table)
    torque_gravity_history = []
    torque_external_force_history = []
    tau_history = []
    p_function = potential_energy(link_masses, link_centers, transforms)
    print(p_function)

    # Starting Position and Drawing Target
    robot_start = compute_forward_position_kinematics(q_function, dh_table, joint_values)
    draw_start = robot_start.copy()
    draw_start[0] += 1
    draw_start[1] += 1
    draw_start[2] += 1
    verbose and print(f'Robot arm starting position is {robot_start}')
    verbose and print(f'Starting drawing at {draw_start}')

    # Shape Maker
    model_collection = [
        model_definition0(circle_radius),
        model_definition1(circle_radius),
        model_definition2(circle_radius)
        ]
    model_defs = model_collection[model_number]
    model_lengths = model_path_lengths(model_defs)
    model_times = model_path_times(model_lengths, draw_time)

    path_functions, path_waypoints, path_times = shape_paths(model_defs, model_times, robot_start, draw_start,
                                                              startup_time)

    q_goal_functions = path_functions
    q_dot_goal_functions = []

    for i in range(len(q_goal_functions)):
        q_dot_goal_functions.append(q_dot_end_effector(q_goal_functions[i]))

    t_range = round(sum(path_times) * time_scale) + 1

    for i in range(0, t_range, time_delta):
        # start_time = ttt.perf_counter()
        print("-----------------------------------------------------------------------------------------")
        time = i / time_scale
        print(f"Log for t={time}, dt={dt}")

        # determine path to draw and its location
        n, draw_time_offset = path_picker(time, path_times)

        # determine q_goal for plot
        q_goal = compute_q_end_effector(q_goal_functions[n], path_waypoints[n], draw_time_offset)
        # verbose and print(f'End Effector Goal Function: {q_goal_functions[n]}')
        verbose and print(f'End Effector Goal Position: {round_matrix(q_goal)}')

        # q function and values (q_function pre-computed)
        q = compute_forward_position_kinematics(q_function, dh_table, joint_values)
        verbose and print(f'End Effector Current Location: {round_matrix(q)}')

        q_delta = q_goal - q
        verbose and print(f'End Effector Position Error: {round_matrix(q_delta)}')

        # end-effector goal q_dot
        q_dot_goal = compute_q_dot_end_effector(q_dot_goal_functions[n], draw_time_offset)
        verbose and print(f'End Effector Goal Velocity: {round_matrix(q_dot_goal)}')

        verbose and print(f'End Effector Goal Velocity (corrected): {round_matrix(q_dot_goal)}')

        # Jacobian and Jacobian Inverse
        jj = jacobian_evaluated(jl_function, dh_table, joint_values, transforms)

        jj_condition = np.linalg.cond(np.array(jj).astype(float))
        verbose and print(f'Jacobian Condition Number: {jj_condition}')
        effective_lambda = lambda_factor
        if use_scaled_lambda:
            effective_lambda *= jj_condition
        verbose and print(f'Lambda Factor: {lambda_factor}')

        jj_inv = jacobian_inverse_dls(jj, effective_lambda)

        # Dynamics
        torque_gravity = compute_joint_torques2(p_function, dh_table, joint_values)
        torque_gravity_history.append(torque_gravity)
        verbose and print(f'Joint Torques: {smp.Matrix(torque_gravity)}')

        external_force = compute_external_force(end_effector_force, jj)
        torque_external_force_history.append(external_force)
        verbose and print(f'External Force Torques: {external_force}')

        tau = torque_gravity - external_force
        tau_history.append(tau)
        verbose and print(f'τ (tau): {tau}')

        # thetas (joint values) and theta_dots
        theta_dots = compute_theta_dots(jj_inv, q_dot_goal)
        verbose and print(f'theta_dots: {round_matrix(theta_dots)}')

        # add to plot data
        path_data.append(q)
        if time > path_times[0]:
            goal_data.append(q_goal)

        # increment joint values for next loop
        joint_values_history.append(joint_values.copy())
        joint_values = increment_thetas(joint_values, theta_dots, dt)
        verbose and print(f'New Joint Values: {round_matrix(smp.Matrix([joint_values]))}')

    plot(path_data, goal_data, draw_time, dt, 'xy')
    plot(path_data, goal_data, draw_time, dt, 'yz')
    plot3d(path_data, goal_data, draw_time, dt, joint_values_history, dh_table, plot_elevation, plot_azimuth)
    #plot_torque_over_time(torque_gravity_history, draw_time, dt, 'Torque due to Gravity, g(q)')
    #plot_torque_over_time(torque_external_force_history, draw_time, dt, 'Torque due to External Force')
    #plot_torque_over_time(tau_history, draw_time, dt, 'Tau Force')
    return

def main():
    #Vertical Plot on YZ plane
    joint_values_1 = [0.0001, 1.0001, 1.0001, 0.0001, 0.0001, 0.0001]
    run_simulation(0, 0.01, False, 0, -90, joint_values_1)

    #Horizontal Plot on XY plane at Z=0
    # joint_values_2 = [0.0202, -1.1763, -1.5412, 1.2181, -1.5675, 0.0]
    # run_simulation(2, 0.001, True, 15, 80, joint_values_2)


if __name__ == "__main__":
    main()
