import matplotlib.pyplot as plt

import Robot


def main():
    robot = Robot.Robot()
    robot.draw()


if __name__ == "__main__":
    plt.xlim([-5, 5])
    plt.ylim([-5, 5])
    main()
    plt.show()

