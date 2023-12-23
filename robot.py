import numpy as np


class RobotState:
    def __init__(self, robot_id, initial_position=(0, 0, 0), initial_yaw_angle=0,
                 initial_velocity=(0, 0, 0), initial_acceleration=(0, 0, 0)):
        self.robot_id = robot_id
        self.position = np.array(initial_position)  # 机器人位置 (x, y, z)
        self.yaw_angle = initial_yaw_angle  # 机器人偏航角
        self.velocity = np.array(initial_velocity)  # 机器人速度 (vx, vy, vz)
        self.acceleration = np.array(
            initial_acceleration)  # 机器人加速度 (ax, ay, az)

    def update_position(self, new_position):
        self.position = np.array(new_position)

    def update_yaw_angle(self, new_yaw_angle):
        self.yaw_angle = new_yaw_angle

    def update_velocity(self, new_velocity):
        self.velocity = np.array(new_velocity)

    def update_acceleration(self, new_acceleration):
        self.acceleration = np.array(new_acceleration)

    def __str__(self):
        return f"Robot {self.robot_id} - Position: {self.position}, Yaw Angle: {self.yaw_angle}, " \
               f"Velocity: {self.velocity}, Acceleration: {self.acceleration}"


# 示例用法
robot1 = RobotState(robot_id=1)
print(robot1)

# 更新机器人状态
robot1.update_position((1, 2, 0))
robot1.update_yaw_angle(np.pi / 4)
robot1.update_velocity((2, 1, 0))
robot1.update_acceleration((1, 1, 0))
print(robot1)
