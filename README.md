# Summary

This simulation environment is for project of 2023 version of Multiple Robots Control Class in SJTU, mainly for question
3, and 4.

The project is to implement a multi-robot system to validate some algorithms which can maintain connections between 
robots in a group. The algorithms are proposed by the paper "On Decentralized Connectivity Maintenance for Mobile 
Robotic Systems" by Lorenzo Sabattini, Nikhil Chopra and Cristian Secchi.

The code structure is well-designed and well-documented. I just implement the basic requirements of the project, but 
there are many class attributes and methods designed but not used in final demo. And some parameters of the simulation 
is not the best, but that's enough for my demo. If you'd like to improve this project, you can add more features to it 
or find a better parameter set.

# Code Structure

## `Robot` class

This class is the basic class of the simulation. It contains the basic attributes and methods of a robot. The `Camera` 
module fixed on the robot is individual from `Robot` class, so actually there are two parameters affecting the visible 
region of the robot.

But in the demo I didn't use the angle of the `Camera` module, and the rotating speed of the robot is not the camera's 
rotating speed. It's simplified to make the demo easier. If you'd like, you can have both camera rotating speed and 
robot rotating speed.

By the way, the way to get a group of robots is through the static method `Robot.initialize_group()`. I think it's 
better to define another class called `RobotGroup` to manage the group of robots, but I haven't done that. If you'd
like, you can implement that.

### `SeeFarthestOneRobot` class

The `Robot` class can communicate with all other robots which are in its visible region. That doesn't fit the 
requirements of the project, so I just inherit this class to get `SeeFarthestOneRobot` class. And the only method need 
to override is `get_visible_robots`.
Here I choose to see the farthest robot in the visible region, but you can also choose to see the nearest one or the one
with the highest priority, just try with your creativity if you'd like.

### `DoubleIntegralRobot` class

This class is the basic class of the robot in question 4. It inherits `SeeFarthestOneRobot` class and override the 
`update` method. Now this method accepts acceleration, and update the posture of the robot by double integral.

## `Controller` class

This class is the basic class of the controller. It contains the basic attributes and methods of a controller, including
the `Robot` group it controls, ranges of speed, acceleration, angular velocity, and the round methods of these. 

The most important method is `speed_gen`, which is used to generate the speed of the robots in the form of `[(speed_x, 
speed_y, angular_velocity), ...]`. 
To implement control algorithms, you only need to override this method and keep the APIs unchanged.

### `CircularTraceController` class

This class inherits `Controller` class. With requirements of circular trace, it adds some attributes and override some 
methods which is basic for a circular trace controller.

Considering the actual control of robots in real world, the way for updating robots position is by x + v * dt, not by 
w * r * dt. But this method has a problem, the robots will definitely move off the circle.
So I add a method `speed_adjust` to adjust the speed of the robots to make them move on the circle. The main idea here 
is to add a velocity component along the radius of the circle according to the difference between the robots' position 
and the radius of the circle. This velocity component depends on the velocity along the trace, and controlled by 
parameter `k`. If this velocity component is too large, the robot will oscillate along the trace.
This method is not perfect, but it works. If you have better algorithms, you can add them here.

The `adjust_ave` method is used to adjust the average speed of the robots moving along the trace, this is required in 
the project requirements.

### `CentralController` class

This class inherits `Controller` class. It just overrides the `speed_gen` method and give speeds based on the 
centralized control algorithm. But in this demo, I didn't implement any control algorithm, just validate the code when 
during the developing process. 

### `DecentralizedController` class

This class inherits `DecentralizedController` class. This is where I implement the decentralized control algorithm 
mentioned in the paper. 
Decentralized controller needs Laplacian matrix of the graph, so I added a new attributes `matrix` for storing Laplacian
matrix, and update it by manually calling `update_l_matrix` method. 
I tried to make this method do it automatically by write this method as property, but found that if this matrix is used 
multiple times in one iteration, the matrix will be updated multiple redundant times and slow down the simulation. So I 
just cache the matrix and update it manually. If you have better ideas, you can change it.

To implement the algorithms, I added some helper functions to make the code more readable and keep the `speed_gen` 
method simple and clear. Maybe they should be private, but to implement `EclipseTraceController` with little code, I 
just make them public.

### `DoubleIntegralController` class

This class inherits `DecentralizedController` class and is for controlling the double integral robots model in 
question 4. It just overrides the `speed_gen` method and give acceleration instead of speed.

The algorithm to generate control acceleration is simple, just calculate the difference between the current speed and 
desired speed according to the algorithm implemented in `DecentralizedController` and multiply with a gain coefficient
`__k_acc`. This algorithm has a problem that the robots will oscillate if `__k_acc` is large, and because we can't 
control the moving speed, the robots will move slowly, so can't reach the average speed requirement. But it's enough for
this demo. If you have better algorithms, you can try them here.

### `EclipseTraceController` class

This class inherits `DecentralizedController` class and is for controlling the robots to trace an eclipse in question 4.
Here I use the method of affine transformation. To get the speed for tracing eclipse trace, I first get the speed for
tracing a circle, and then transform the speed to the speed for tracing an eclipse. Just multiply with y component of 
the speed with coefficient `b/a`, `b` is half of the minor axis of the eclipse, `a` is half of the major axis of the
eclipse.

I think maybe it's better to implement a `EcpliseTraceController` class and then implement `DecentralizedController` for
`EclipseTraceController` class, but I have no time. If you'd like, you can implement that.

## `Simulation` class

This class has four main components, `robots`, `controller`, `ani` for storing the animation instance, and single `fig` 
and `ax` for plotting this instance of simulation. 
I do animation with `matplotlib` and `matplotlib.animation`, more accurately, I use 
`matplotlib.animation.FuncAnimation`. 
To run the animation, three basic methods are required based on the official document. With this `show_animation` 
method, you can simulate multiple scene at the same time, just by creating multiple instances of `Simulation` class.

## `main.py`

In main function, I created three instances of `Simulation` class. Note that the initial position of robots for eclipse
trace is not along the trace, but that's fine because it'll converge to the trace soon.


