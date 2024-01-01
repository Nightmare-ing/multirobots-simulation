import math

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from Parts import Robot, SeeFarthestOneRobot, DoubleIntegralRobot
from Controller import CentralController, DecentralizedController, DoubleIntegralController, EclipseTraceController
from Simulation import Simulation


def equal_space_posture(num_robots, radius=5.0):
    interval_angle = 2 * math.pi / num_robots
    start_angle = math.radians(90) + interval_angle / 2
    return [(radius * math.cos(start_angle + i * interval_angle), radius * math.sin(start_angle + i * interval_angle),
             math.radians(90) + i * interval_angle) for i in range(num_robots)]


def link_posture(num_robots, radius=5.0):
    interval_angle = 2 * math.pi / num_robots / 1.5
    start_angle = math.radians(90) + interval_angle / 2
    return [(radius * math.cos(start_angle + i * interval_angle), radius * math.sin(start_angle + i * interval_angle),
             math.radians(90) + i * interval_angle) for i in range(num_robots)]


def main():
    robots_equal_spaced = SeeFarthestOneRobot.initialize_group(4, equal_space_posture(4),
                                                               speeds=[(0, 0, 0)] * 4)
    robots_link = SeeFarthestOneRobot.initialize_group(4, link_posture(4),
                                                       speeds=[(0, 0, 0)] * 4)
    robots_equal_spaced1 = DoubleIntegralRobot.initialize_group(4, equal_space_posture(4),
                                                                speeds=[(0, 0, 0)] * 4)

    # dec means decentralized, cen means centralized
    fig_dec_circle, ax_dec_circle = plt.subplots()
    fig_dec_link, ax_dec = plt.subplots()
    fig_dec_eclipse, ax_dec_eclipse = plt.subplots()

    controller_dec = DecentralizedController(robots_link)
    scene_dec = Simulation(fig_dec_link, ax_dec, robots_link, controller_dec)
    scene_dec.show_animation()

    # controller = CentralController(robots)
    controller_double_integral = DoubleIntegralController(robots_equal_spaced1)
    scene_double_integral = Simulation(fig_dec_circle, ax_dec_circle, robots_equal_spaced1, controller_double_integral)
    scene_double_integral.show_animation()

    controller_eclipse_trace = EclipseTraceController(robots_equal_spaced)
    scene_eclipse = Simulation(fig_dec_eclipse, ax_dec_eclipse, robots_equal_spaced, controller_eclipse_trace)
    scene_eclipse.show_animation()
    # simulation_scene.save_animation("test.mp4")
    plt.show()


if __name__ == "__main__":
    main()
