import numpy as np


# caculate quintic_polynomial a
def quintic_polynomial_coeffs(p0, v0, a0, p1, v1, a1, t0, tf):
    T = tf - t0
    M = np.array([
        [1, t0, t0**2, t0**3, t0**4, t0**5],
        [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
        [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
        [1, tf, tf**2, tf**3, tf**4, tf**5],
        [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
        [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
    ])
    b = np.array([p0, v0, a0, p1, v1, a1])
    a = np.linalg.solve(M, b)
    return a


def quintic_polynomial_trajectory(a, t):
    # T = np.array([1, t, t**2, t**3, t**4, t**5])
    # return np.dot(a,T)
    return np.polyval(a[::-1], t)

def quintic_polynomial_velocity(v, t):
    # T = np.array([0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4])
    # return np.dot(a,T)
    for i in range(len(v)):
        v[i] = v[i] * i
    return np.polyval(v[:0:-1], t)


def polynomial_trejectory_planing(waypoints_position, time_stamps):
    """使用五次多项式规划生成位置和速度的轨迹

    Args:
        waypoints_position (NDArray[[]]): 二维矩阵，行代表途径点的个数，列代表关节的维度
        time_stamps (NDArray[[]]): 一维矩阵，代表途径各个点的时间
    Returns:
        _type_: 返回轨迹的位置和速度
    """
    #velocity
    initial_vel = np.zeros(7)
    final_vel = np.zeros(7)
    #accelerate
    initial_acc = np.zeros(7)
    final_acc = np.zeros(7)

    coeffs_position = []


    for i in range(len(waypoints_position) - 1):
        p0 = waypoints_position[i]
        t0 = time_stamps[i]

        p1 = waypoints_position[i + 1]
        tf = time_stamps[i + 1]

        #position x/y/z
        segment_coeffs_position = [quintic_polynomial_coeffs(p0[j], initial_vel[j], initial_acc[j], p1[j], final_vel[j], final_acc[j], t0, tf) for j in range(7)] #手臂七个关节
        coeffs_position.append(segment_coeffs_position)

        
    trajectory_position = []
    trajectory_velocity = []

    for i in range(len(waypoints_position) - 1):
        t0 = time_stamps[i]
        tf = time_stamps[i + 1]
        time_samples = np.linspace(t0, tf, num=100*(tf-t0))
        segment_trajectory_position = np.array([quintic_polynomial_trajectory(coeffs_position[i][j], time_samples) for j in range(7)]).T
        segment_trajectory_velocity = np.array([quintic_polynomial_velocity(coeffs_position[i][j], time_samples) for j in range(7)]).T
        trajectory_position.append(segment_trajectory_position)
        trajectory_velocity.append(segment_trajectory_velocity)

    trajectory_position = np.vstack(trajectory_position)
    trajectory_velocity = np.vstack(trajectory_velocity)
    return trajectory_position ,trajectory_velocity