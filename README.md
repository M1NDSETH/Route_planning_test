This program builds the shortest route to the targets with its subsequent visualization and the output of the route length in the coordinates of the program grid.
All object coordinates are specified in the program grid, which is not tied to real coordinates.

## CLASSES

There are two classes implemented in this program:
1) *GRID* - contains info about size of grid (*x_size, y_size*), coordinates of targets (*targets*) in this grid.
2) *AUV* - class for vehicle, contains info about coordinates of vehicle's starting point (*start_point*) and class method for building a route (*build_full_route*)

## START OF PROGRAM

Before starting the program, the user sets the coordinates of the starting point of the device (*start_point*), the size of the grid of the program (*grid_size_x, grid_size_y*), the coordinates targets.

## PROGRAM OPERATION

After starting work, the program builds the shortest route from the starting point to the targets using the A* algorithm. By default the device can move in 8 directions (*directions* list): 4 along the X and Y axes and 4 at an angle of 45 degrees to the axes. The device moves ONLY through cells with integer coordinates in the grid.

## CUSTOMIZATION

The user can set the starting point of the device, the positions of the targets, the dimensions of the grid and install the internal walls in the movement space. The wall is set by replacing the cell values with the corresponding coordinates from 0 to 1.

## VISUALIZATION

The red line on the graph shows the route of the device, the black squares are the walls.