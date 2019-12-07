# AWS Deepracer Optimal Path Generator

This repository presents the method used by the French Deepracer team for the world final at the Reinvent in Las Vegas. 
Our approach was to generate the optimal path of an AWS Deepracer track given a certain set of waypoints.



##  STEP 1 : Generate a first set of custom waypoints

Run the generate_track.py file to generate the track, and select your track using inquierer (for Windows users, you have to modify the code to select the the track you want. Cf init function in the generate_track.py class).  

As the A* and the rear wheel feedback only accept integers, we have to multiply all coordinates by a coefficient that you must enter, then, you need to interpolate all waypoints to build the barriers. 14 and 5 should works pretty well for most of the track. 
Once you have enter these two values, a plot should open and a* start and reer wheel will be running once you have closed the plot window.

#### Plot of the track 
![Plot](https://github.com/matrousseau/AWS-Deepracer-Optimal-Path-Generator/blob/master/CubicSpline/IMG/plot%20of%20the%20track.png)

#### A start & Rear wheel
![A*](https://github.com/matrousseau/AWS-Deepracer-Optimal-Path-Generator/blob/master/CubicSpline/IMG/a*.png)

![Rear wheel](https://github.com/matrousseau/AWS-Deepracer-Optimal-Path-Generator/blob/master/CubicSpline/IMG/rear%20wheel%20feedback.png)

##  STEP 2 : Improve the waypoints quality

Open the jupyter notebook and load the waypoints generated in the data folder. 



