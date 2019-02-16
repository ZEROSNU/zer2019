Header header
geometry_msgs/Vector3[] initpoints #list of starting points of lines of parking lot, the nearest one listed first , in m coordinate
geometry_msgs/Vector3 goal_point #goal point(middle) of the parking lot, if len(initpoints)>=3, the goal point is the middle of the nearer parking lot
