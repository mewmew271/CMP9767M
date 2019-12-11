# CMP9767M

This assignment is mainly focused on navigation with some aspects of vision 

Debug code can be uncommented to allow for more data to be seen. 

Operational requirements 
Thorvald simulation and move base (imutils should be installed with catkin make)

Summary of operation Robot will move to preset way point representing each row of crops. Checks for weeds using an hsv threshold to make a mask, the mask is then denoised and blurred. 
Objects are then Identified by looking for contours in the mask and the centroids are marked. Centroid coordinates are transformed into world coordinates and checked to see if they have been moved too.
Robot moves to each coordinate then back to its previous location to regain its position in the row. After all points have been moved to the robot will move to a waypoint and repeated.
