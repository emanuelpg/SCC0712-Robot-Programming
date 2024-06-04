# SCC0712-Robot-Programming
Project of the naviagation algorithm of a robot, with obstacle avoidance, to reach a goal point on the map.

The following program consists of a robot's navigation system with two features: reaching a fixed objective location with a determined position; and avoid obstacles that appear on the way. To achieve this, the robot is equipped with a GPS and a LiDAR sensor.
The strategy to reach the objective consists of finding the robot direction vectors and the objective position vector (with robot relation). This way, the angle between these two vectors is found, through the scalar product form, and thus we know how much the robot must rotate to reach its goal.
To determine the robot's direction vector, the robot's current position and its previous position (determined by a number of time steps), that is, by subtracting the points in question. To determine the position vector of the objective, subtract its position from the current position of the robot.
To assist with the time in which the robot must rotate, a delay function was created which, with the angle parameter and a multiplicative factor of 1.1 (optimized by trial and error), allows the robot to maintain rotation for a period of correct time.
Another important aspect is that when encountering an obstacle, the robot prioritizes avoiding that obstacle and then continuing its search for the objective. This feature presents room for improvement (which would be to enable the robot to avoid the obstacle and move towards its goal at the same time), but at first it appears to be satisfactory.
