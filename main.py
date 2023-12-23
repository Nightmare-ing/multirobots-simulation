import matplotlib.pyplot as plt

import Robot


def main():
    robot = Robot.Robot()
    robot1 = Robot.Robot()
    fig, ax = plt.subplots()
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.autoscale(enable=False)
    # draw the animation of robot moving in x direction with velocity 1 with fixed canvas
    for i in range(100):
        robot.update_position((i / 10.0, 0, 0))
        robot1.update_position((0, i / 10.0, 0))
        robot.draw(ax)
        robot1.draw(ax)
        plt.pause(0.1)
        plt.cla()


if __name__ == "__main__":
    main()
    plt.show()



