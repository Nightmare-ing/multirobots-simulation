import matplotlib.pyplot as plt

import Robot
import Central_Controller


def main():
    robots = initialize_robots(4,
                               positions=[(10, 0, 0), (0, 10, 0), (-10, 0, 0), (0, -10, 0)],
                               speed=[(0, 0), (0, 0), (0, 0), (0, 0)])
    fig, ax = plt.subplots()
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.autoscale(enable=False)

    controller = Central_Controller.Controller(robots, fig)
    controller.run()


def initialize_robots(num_robots, positions, speed):
    robots = [Robot.Robot(initial_posture=positions[i], speed=speed[i]) for i in range(num_robots)]
    return robots


if __name__ == "__main__":
    main()
    plt.show()



