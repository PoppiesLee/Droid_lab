import os
import sys
import time
import numpy as np
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), './'))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../protos'))


class NanoSleep:
    def __init__(self, ms):
        self.duration_sec = ms * 0.001  # 转化为单位秒

    def waiting(self, _start_time):
        while True:
            current_time = time.perf_counter()
            elapsed_time = current_time - _start_time
            if elapsed_time >= self.duration_sec:
                break

def get_command(last_value, current_value, max_increment):
    """
    返回一个值，该值满足最大增量限制。

    :param last_value: 上一个值
    :param current_value: 当前值
    :param max_increment: 最大增量
    :return: 限制后的值
    """
    # 计算当前值与上一个值之间的差值
    increment = current_value - last_value
    # 如果差值的绝对值超过了最大增量，则限制它
    if abs(increment) > max_increment:
        # 如果差值为正，则增加最大增量；如果差值为负，则减少最大增量
        return last_value + (max_increment if increment > 0 else -max_increment)
    # 如果差值在允许范围内，则直接返回当前值
    return current_value

def quaternion_to_euler_array(quat):
    # Ensure quaternion is in the correct format [x, y, z, w]
    x, y, z, w = quat
    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)
    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch_y = np.arcsin(t2)
    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)
    # Returns roll, pitch, yaw in a NumPy array in radians
    return np.array([roll_x, pitch_y,yaw_z])  # , yaw_z

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion (w, x, y, z).
    :param roll: Rotation around X-axis (in radians)
    :param pitch: Rotation around Y-axis (in radians)
    :param yaw: Rotation around Z-axis (in radians)
    :return: Quaternion as a tuple (w, x, y, z)
    """
    # Compute half angles
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    # Calculate quaternion components
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [w, x, y, z]

def quat_rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate a vector by the inverse of a quaternion along the last dimension of q and v.

    Args:
        q: The quaternion in (w, x, y, z). Shape is (4,).
        v: The vector in (x, y, z). Shape is (3,).

    Returns:
        The rotated vector in (x, y, z). Shape is (3,).
    """
    v = np.array(v)
    q_w = q[0]
    q_vec = q[1:]
    a = v * (2.0 * q_w ** 2 - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a - b + c

def init_command(command, num_actions):
    for idx in range(num_actions):
        command.mode.append(1)
        command.position.append(0.0)
        command.velocity.append(0.0)
        command.torque.append(0.0)
        command.ens.append(1)
        command.kp.append(0.1)
        command.kd.append(0.1)
        command.max_torque.append(1)


def set_motor_mode(command, config):
    idx_max = len(config.joint_name)
    command.cmd_enable = 1
    for idx in range(idx_max):
        command.kp[idx] = config.kp[idx]
        command.kd[idx] = config.kd[idx]
        command.max_torque[idx] = config.imax[idx]


def set_joint_mode(command, config, num_actions):
    # command.cmd_enable = 2
    for idx in range(num_actions):
        command.kp[idx] = config.dof_stiffness[idx]
        command.kd[idx] = config.dof_damping[idx]
        command.max_torque[idx] = config.effort_limit[idx]

def set_joint_mode_E1(command, config, num_actions):
    command.cmd_enable = 2
    command.kp[0:3] =   config.dof_stiffness[0:3]
    command.kp[3] =     config.dof_stiffness[3]
    command.kp[4] =     config.dof_stiffness[3]
    command.kp[5:7] =   config.dof_stiffness[4:6]
    command.kp[7:10] =  config.dof_stiffness[6:9]
    command.kp[10] =    config.dof_stiffness[9]
    command.kp[11] =    config.dof_stiffness[9]
    command.kp[12:14] = config.dof_stiffness[10:12]

    command.kd[0:3] =   config.dof_damping[0:3]
    command.kd[3] =     config.dof_damping[3]
    command.kd[4] =     config.dof_damping[3]
    command.kd[5:7] =   config.dof_damping[4:6]
    command.kd[7:10] =  config.dof_damping[6:9]
    command.kd[10] =    config.dof_damping[9]
    command.kd[11] =    config.dof_damping[9]
    command.kd[12:14] = config.dof_damping[10:12]

    command.max_torque[0:3] =   config.effort_limit[0:3]
    command.max_torque[3] =     config.effort_limit[3]
    command.max_torque[4] =     config.effort_limit[3]
    command.max_torque[5:7] =   config.effort_limit[4:6]
    command.max_torque[7:10] =  config.effort_limit[6:9]
    command.max_torque[10] =    config.effort_limit[9]
    command.max_torque[11] =    config.effort_limit[9]
    command.max_torque[12:14] = config.effort_limit[10:12]

def print_configs(config):
    idx_max = len(config.joint_name)
    line = '-' * idx_max * 11
    print("---------+" + line)
    print("MtrName  |", end="")
    for idx in range(idx_max):
        print(f"{config.joint_name[idx]:>11}", end="")
    print()
    print("---------+" + line)
    for attr in ["pzero", "pmin", "pmax", "imax", "kp", "kd"]:
        print("{:<8} |".format(attr.upper()), end="")
        for i in range(idx_max):
            print("{:>11.3f}".format(getattr(config, attr)[i]), end="")
        print()

    print("---------+" + line)


def print_state(state, config):
    idx_max = len(config.joint_name)
    line = '-' * idx_max * 11
    print(f"system tic: {state.system_tic} ms")
    print("---------+" + line)
    print("MtrName  |", end="")
    for i in range(idx_max):
        print(f"{config.joint_name[i]:>11}", end="")
    print()
    print("---------+" + line)

    labels = ["qc", "dqc", "tqc", "temp", "absc", "loss"]
    for label in labels:
        print(f"{label:<8} |", end="")
        for i in range(idx_max):
            if label == "qc":
                print(f"{state.position[i]:>11.3f}", end="")
            elif label == "dqc":
                print(f"{state.velocity[i]:>11.3f}", end="")
            elif label == "tqc":
                print(f"{state.torque[i]:>11.3f}", end="")
            elif label == "temp":
                print(f"{state.temperature[i]:>11.3f}", end="")
            elif label == "absc":
                print(f"{state.abs_encoder[i]:>11.3f}", end="")
            elif label == "loss":
                print(f"{state.pack_loss[i]:>11}", end="")
        print()
    print("---------+" + line)
    line = '-' * 80
    print(
        f"Foot Sensor (L L R R):      {state.foot_force[0]:>10.3f} {state.foot_force[1]:>10.3f} {state.foot_force[2]:>15.3f} {state.foot_force[3]:>10.3f}")
    print(line)

    print(f"Imu pack stamp: {state.imu_stamp:<10}")
    print(
        f"Accelerometer (m/s^2): {state.imu_acc[0]:>17.3f} {state.imu_acc[1]:>17.3f} {state.imu_acc[2]:>17.3f}")
    print(
        f"Attitude      (Euler): {state.imu_euler[0]:>17.3f} {state.imu_euler[1]:>17.3f} {state.imu_euler[2]:>17.3f}")
    print(
        f"Gyroscope     (rad/s): {state.imu_gyro[0]:>17.3f} {state.imu_gyro[1]:>17.3f} {state.imu_gyro[2]:>17.3f}")
    print(line)

    print(
        f"Attitude(est) (Euler): {state.est_euler[0]:>17.3f} {state.est_euler[1]:>17.3f} {state.est_euler[2]:>17.3f}")
    print(
        f"COM Pos(est)      (m): {state.est_com_pos[0]:>17.3f} {state.est_com_pos[1]:>17.3f} {state.est_com_pos[2]:>17.3f}")
    print(
        f"COM Vel(est)    (m/s): {state.est_com_vel[0]:>17.3f} {state.est_com_vel[1]:>17.3f} {state.est_com_vel[2]:>17.3f}")
    print(line)

    print(
        f"Bus Information:      {state.bus_voltage:>18.3f} {state.bus_current:>17.3f} {state.bus_energy:>17.3f}")
    print(line)

    print(
        f"Remote Controller: {state.rc_du[0]:>21} {state.rc_du[1]:>10} {state.rc_du[2]:>13} {state.rc_du[3]:>10}")
    print(
        f"{state.rc_keys[0]:>40} {state.rc_keys[1]:>10} {state.rc_keys[2]:>13} {state.rc_keys[3]:>10}")
    print(line)