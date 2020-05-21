# sample_movement

This initially demonstrated moving a turtlebot with simple c++ code, and that can still be seen in the sample_movement and print c++ files in src. 
The other files are working on a plugin for move_base, to replace the global path planner. It consists mostly of rrt, which is the path planner logic (still unfinished/buggy) and graph, which is a temp graph structure that contains neccessary but inefficent functionality.
Running ubuntu 16.04 LTS on vbox 6 windows host, with ROS Kinetic as installed from Robotis emanuel

Instructions:

-modiy the CMakelist-the one in this repository should work correctly

-package.xml does not need to be changed

-```catkin_make``` to compile the package

-```roscore```

-launch the regular launch file on the turtlebot


-```rosrun sample_movement sample_movement```



adapted from [this link](https://github.com/demulab/move) and [here](https://github.com/jeshoward/turtlebot_rrt)

Built while working in Dr.Ottes lab; University of Maryland
