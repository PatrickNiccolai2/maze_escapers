# First, update the dockerfile to include necessary packages

add `python-tk` to docker file under `RUN apt-get -y update && apt-get install -y \` in order for maze_reader to work

Also add `RUN pip install --upgrade scipy` after the apt section

The dockerfile in this repo has all necessary changes

# World files

The two world files we used for the experiments are smallmaze3.world and smallmaze5.world

These files should be in the `vnc-ros/workspace/src/turtlebot_simulator/turtlebot_stage/maps/stage` folder. The turtlebot.inc file in the repo should also be in this folder. If you don't have these folders, the package is here: https://github.com/turtlebot/turtlebot_simulator

The smallmaze.png should be in the `vnc-ros/workspace/src/turtlebot_simulator/turtlebot_stage/maps` folder. 

# Running the program

In the docker environment, first run `roscore` in a terminal window. To create the world, run `rosrun stage_ros stageros /root/catkin_ws/src/turtlebot_simulator/turtlebot_stage/maps/stage/smallmaze3.world`. Or replace `smallmaze3.world` with `smallmaze5.world` for the world with 5 robots. 

Run `rosrun maze_escapers shared_map` to start the shared_map node. Then run `ROS_NAMESPACE=robot_0 rosrun maze_escapers explorer` to create the first explorer node, `ROS_NAMESPACE=robot_1 rosrun maze_escapers explorer` to create the next explorer node, etc. Alternatively, `./three_explorers.sh` will run all the nodes for shared_map and three explorers and `./five_explorers.sh` will run all the nodes for shared_map and five explorers. `./kill10.sh` will stop the nodes. 

Finally, note that num_robots on line 26 of the shared_map file should be changed to 3 or 5 according to the number of robots you are running.