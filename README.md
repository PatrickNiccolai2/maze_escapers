# Auctions package

## To run

It is assumed that there are 10 robots with names robot_0, robot_1, etc. It is also assumed that robot_0 will run the leader node, and the rest will run the follower node. 

I used the stage world file `stage_empty_with_robots.world`

First, run `rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map world 5` to create the `world` reference frame.

Then, run `rosparam set num_robots 10` and `rosparam set num_points 10`, or whatever number of waypoints you want, and `rosparam set com_dist 3`, to set the maximum distance two nodes can communicate, and `rosparam set num_terms 3` to set the number of terminal nodes. 

Then, run `rosrun steiner rand_waypoints` to publish waypoints. Run this after the ROS master is running. 

Then, run `rosrun steiner tf_broadcaster` to start the tf broadcaster.

Then, run the leader and follower nodes with `ROS_NAMESPACE=robot_0 rosrun steiner leader`. The follower nodes can be run with `./run10.sh` and killed with `./kill10.sh`. 