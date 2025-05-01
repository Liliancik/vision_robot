This is my codes used for the Mars Rover competition, it includes the code names listed in the Report which are used to test and set up:

- Waypoint follower (Maybe_W.py) code executes as perfectly as possibple yet no avoidance is incorporated.
- (CRobot.py) Is responsible for the H-Bridge control which can be called and used to move the robot.
- (object_detection_realsense_depth.py) Can be used to test and see that the realsense camera is working showing depth and distance while also boxing the object detected.
- (app_1.py) Lets the user boot up the mapping grid where the points can be plotteted and saved in the same folder as a text file 
- (avoid_idea.py) Is the avoidance sequence where the points are hardcoded from (0,0) to (1,0) works very well, but i didn't get the chance to test it fully togheter with the waypoints implemented from the app_1.py, or aditional points.

To access my codes from raspberry py just follow these steps:
/home/pi/Desktop/Robot/aiy-maker-kit/examples/rssetup

To access them in Github:
vision_robot/aiy-maker-kit/examples/rssetup
