# SYSTEM DESIGN THINKING of Pick and Place framework using MoveIt ROS.

ROS environment for ___practicing___ MoveIt pick and place with Panda Robot

### State transitions

![State transition flowchart](/fig/actionserver_flowchart.png "State transitions")

### Pick and place environment

![Pick and place environment](/fig/environment.png "Pick and place environment")


### MoveIt pick and place server

- [x] Verified that the included launch file loads MoveIt RViz with the Panda robot and backyard environment
- [x] Implemented actionlib server to receive requests from the state machine
- [x] Implemented MoveIt interface to process requests (create motion plans, and execute valid motions)
  - [x] Minimum required actions requests are to either Move to pose, Pick at pose, or Place at pose
  - [x] The minimum desired input is the target pose of the robot end-effector
  - [x] The minimum desired output is the success status of the plan and execution, however, it is also beneficial to know why the server failed rather than a simple “yes or no” reply
  - [x] The desired usage is to make a call to the actionlib server with the target pose and have the server plan motion, and if it succeeds, execute the motion on the robot.

### State machine

* Created a new ROS package in the workspace and configure the xml and CMake to include any required libraries (ROS and/or external).
- [x] Designed the state transitions so that the previous state’s information is passed into next state
  * _e.g._ Recognized marker state passes it's result to Recognize object, which then passes to Pick object, etc.
- [x] Implemented dummy functionality for Marker Recognition, Object Recognition, and Placement recognition servers
  - [x] For these servers, please still implement actionlib client/server for them. The execution call of the server will simply be a dummy method like sleeping for X seconds etc.
  - [x] With this in mind, It is taken into consideration what meaningful I/O might be for these servers (e.g. at least a marker ID for recognize marker action)
- [x] Implemented actionlib client to send requests to the moveit_pnp-server
- [x] Tested state transitions and completion of MoveIt server requests

## Dependencies

* OS: Ubuntu 18.04
* ROS melodic (see here for install instructions: <http://wiki.ros.org/melodic/Installation/Ubuntu>)

## Clone the Repository

```bash
git clone --recurse-submodules git@github.com:sh1509/moveit_pnp_actionlib_ws.git
```
*__NOTE__ "--rescurse-submodules" as the repository is dependent on submodule panda_moveit_config.

## How to setup

* Install dependencies that are defined in each package.xml (run this from the top folder of the workspace)

```bash
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src -y
```

## How to build

* You should be at the top directory of this project to build this project by catkin build

```bash
catkin build -c
source devel/setup.bash
```
* __NOTE__ You might need to run catkin build twice as all packages except panda_moveit_config are dependent on package "action_msg".

## How to run

* Launch the robot with MoveIt RViz and Panda Robot with backyard environment
```bash
roslaunch moveit_pnp_server moveit_pnp_server.launch
```
* __NOTE__: This launch file contains all the action servers and action client needed to move the object from the stock shelf to the display shelf.


## Limitations, assumptions, and other notes
* __TODO__ Return feedback to action server(I have not done it) But the idea is to use tf message and return the end effector pose till the time, goal is being processed by the action server.
* __TODO__ Ensure that the state machine is sequential. Right now, that is happening in pick_place_client_node in the done callback and it shuts down the node. Maybe if we don't want to shutdown the node, then we can re-arrange the source code in pick_place_client_node to ensure state machine is sequential. 
* __TODO__ Other dummy action servers, it can be more interactive if I can return the result of these action servers with some meaning such as marker recognition returns the pose of the id, object recognition returns the pose of the object etc.,
* __TODO__ If possible optimise the code and make it more stable and efficient by testing it through edge cases.
* __TODO__ Get the shelf id and display id from rosargs during roslaunch. This will ease up user's interaction with the program requesting for actions of moving an object from stock shelf to display shelf.
* __TODO__ Grasping! Right now the robot is not grasping the object. It is just picking the object and placing it. Grasping requires slight interesting intuition depending on the object shape and size and force with with which the grasping should take place.

* __CORRECTION__ One error was that the orientation of the stock shelf and the display shelf in the config file was not correct.

* __NOTE__ I am not removing any packages of package.xml and CMakeLists as I feel they might be needed for other action servers.

* __ASSUMPTION__ I am using move_group interface to move or pick or place, so for that, the pose(especially to pick or place) is slightly tuned with an understanding that the pose received from the config file is the pose of an id. And as Nathan-san mentioned, this id is the aruco marker on the shelf which might lead to collision.
__NOTE__ that these poses are hence tuned in order for the move group interface to avoid collision but this results in slight deviation from the actual pose. Maybe it needs to be visualized properly for accurate tuning.





