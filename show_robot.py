import matplotlib.pyplot as plt
import numpy as np


class RobotState:
    def __init__(self, robot_id, initial_position=(0, 0), initial_yaw_angle=0):
        self.robot_id = robot_id
        self.position = np.array(initial_position)
        self.yaw_angle = initial_yaw_angle

    def update_position(self, new_position):
        self.position = np.array(new_position)

    def update_yaw_angle(self, new_yaw_angle):
        self.yaw_angle = new_yaw_angle

    def __str__(self):
        return f"Robot {self.robot_id} - Position: {self.position}, Yaw Angle: {self.yaw_angle}"


def draw_robot_and_fov(robot):
    # 用颜色表示不同的机器人
    colors = ['blue', 'green', 'red', 'yellow']

    # 画机器人
    plt.plot(robot.position[0], robot.position[1], 'o', color=colors[robot.robot_id-1], markersize=8,
             label=f'Robot {robot.robot_id}')

    # 画扇形表示视场角
    fov_radius = 2.0  # 可调整视场角半径
    fov_theta = np.linspace(robot.yaw_angle - np.pi/4,
                            robot.yaw_angle + np.pi/4, 100)
    fov_x = robot.position[0] + fov_radius * np.cos(fov_theta)
    fov_y = robot.position[1] + fov_radius * np.sin(fov_theta)

    # 添加蓝、绿、红、黄的阴影,透明度为10%
    plt.fill_between(fov_x, fov_y, color=colors[robot.robot_id-1], alpha=0.1)

    # 画扇形边缘
    plt.plot(fov_x, fov_y, 'r--')

    # 添加从机器人原点到扇形顶点构成的三角形的阴影，透明度为10%
    plt.fill([robot.position[0], fov_x[0], fov_x[-1]],
             [robot.position[1], fov_y[0], fov_y[-1]], color=colors[robot.robot_id-1], alpha=0.1)


# 创建四个机器人
robots = [RobotState(robot_id=i+1, initial_position=(np.cos(i *
                     np.pi/2), np.sin(i*np.pi/2))) for i in range(4)]

# 绘制机器人和视场角
for robot in robots:
    draw_robot_and_fov(robot)

# 设置坐标轴范围
plt.xlim([-3, 3])
plt.ylim([-3, 3])

# 添加图例和标签
plt.legend()
plt.title("Robot Configuration with Colored Field of View and Shadow")
plt.xlabel("X-axis")
plt.ylabel("Y-axis")

# 显示图形
plt.show()
