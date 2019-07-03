# Bug2PathPlanning
Bug 2 algorithm to simulate the turltle bot path planning in ROS and Gazebo

The bug is a Simple algorithm. The robot follows the straight line to its goal, called the m-line. If it encounters any object, it goes around the object until it re-encounters the m-line and then starts following it again. 

This algoirthm was tested on 4 different world layouts in Gazebo, and the Turtle bot was used.

## Explanation

The whole algorithm was implemented in the OutAndBack Class that was used in the previous assignment as well. The algorithm keeps track of the possible states, which are "line"and "object". 

In the "line" state, the robot is just follwoing the m-line.

In the "object" state, the robot is circumbenting an object. While this, the algorithm keeps checking whether it hits the m0line or not and if it does
then it return back to the "line" state. The robot enters the "object" state if it detects a wall infront of it. it then turn right and if there is not a wall
then it moves until the edge of the wall is detected.
This wall follwoing is broken into segments durign which the robot checks the m-line and the proximity to the walls. When the edge is detected, the algorithm updates the orientation of the robot to follow the new wall. 

The robot moves and rotates using the move and rotate class funtions defined. A special checker funtion is often called to see how far it is from the m-line.
Due to noice in rotations, when the robot reaches the goal it calculates its deviation from the goal and move towards it, this step is intentionally left in the end, since the noise accumulates over sucessive rotations and is most pronounced towards the end

## Funtions Overviews

*scan_callback*: processing the kinect data, replaces the nan and any value greater than 3 to 100.
*OutAndBack*: The class where everything happens, init is the main algorithm.
*checker*: checks proximity to the m-line and changes state if on the m-line.
*dist*: gives distences between two points
*rotate*: rotates the robots if the by input angle and its tolerance
*move*: move the robot the specified distance
*stop*: stops the robot
*printer*: prints the the left right and cener depth data and also the positon of the robot
*change*: change sthe state of the robot
*getFollow*: gives the current state of the robot
*get_odom*: from tutorial, gives positon of the robot
*shutdown*: sutdowns the robot
