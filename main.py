import matplotlib.pyplot as plt

import robot


def main():
    robot = my_robot.Robot()
    robot.draw()


if __name__ == "__main__":
    plt.xlim([-5, 5])
    plt.ylim([-5, 5])
    main()
    plt.show()

