## Milestone 1: Line Follower

Our goal for milestone 1 was to have our robot autonomously follow a line in a figure-8 pattern. To implement this ability, we started off by deciding the arrangement of the line sensors to recognize a straight path along a piece of black tape on a white background. After achieving this functionality, we developed our code to enable the robot to traverse a grid of lines and follow a figure-8 pattern. 

## Part 1: Line Detection
For our robot to detect and follow a simple, straight line, we first tested the QRE1113 line sensors to see what values corresponded with light and dark surfaces. With those values, we decided to place two line sensors in the center, spaced apart with a distance slightly greater than the width of the line. Using two sensors instead of four guaranteed that there would be sufficient analog pins. With this design the two values provide information on which way to adjust:

If left sensor detects line (has a value greater than that indicating a dark surface) and right sensor detects line, robot is perpendicular to the line and needs to turn in one direction.
If left sensor is on the line and right sensor is not, robot drifted left and needs to turn slightly right.
If right sensor is on the line and left sensor is not, robot drifted right and needs to turn slightly left.
If both sensors detect light (are not on the line), then the robot is centered on the line. 
If robot is off the line, speed should increase to correct its position faster. 

Figure 1. Placement of line sensors on robot.

Link to video of self-adjusting, line-following robot: [Video](https://youtu.be/CC1JYYBU080) 


## Part 2: Autonomous Figure-8 Movement
The next part of our milestone 1 involved turning and detecting intersections in a grid. An intersection was detected when both light sensors were on a line, indicating that the robot should turn 90 degrees. To move the robot in a figure-8 pattern, we kept track of the direction the robot would turn (x = 1 indicates right turn, x = 0 indicates left turn) and the number of consecutive turns in that direction (variable y). For example, if x = 0, the robot would turn left at an intersection and would keep turning left at intersections until the number of consecutive left turns complete would surpass 4. At that point, y is greater than 4, so the robot would change its turn direction (x = !(x)) and restart its tracking of consecutive turns (y = 0).

Link to video of autonomous figure-8 movement:  [Video](https://youtu.be/ZuVscGUPQMY)
