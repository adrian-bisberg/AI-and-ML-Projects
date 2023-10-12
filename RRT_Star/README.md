## An Implementation of the RRT* pathfinding algorithm in RRT*

### Author: 
Adrian Bisberg

### How to run:
 - To run program, provide at least one additional argument, a text file with cooridnate locations of polygon obstacles. 
 In this file, each coordinate should be listed on a single line, the x and y coordinates seperated by a space. Then to denote
 the next polygon in the file leave one empty line before the next polygon. Each polygon must be specified in a counter-clockwise
 direction.
 - Optionally, include values for a start coordinate, goal coordinate, and goal radius in the following fashion:
 './prog points.txt <start_position_x> <start_position_y> <goal_position_x> <goal_position_y> <goal_radius>'
 if any one of thses are provided, they must all be provided.

 ### For more details
 See my final survey paper for the course where this project was developed.