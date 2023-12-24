import matplotlib.pyplot as plt
from Parts import Robot
from Controller import CentralController


def main():
    robots = Robot.initialize_robots(4,
                                     posture=[(3, 0, 0), (0, 3, 0), (-3, 0, 0), (0, -3, 0)],
                                     speed=[(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)])
    fig = plt.figure()
    ax = fig.add_subplot(111, aspect='equal')

    controller = CentralController(robots, ax)
    controller.run()


if __name__ == "__main__":
    main()
