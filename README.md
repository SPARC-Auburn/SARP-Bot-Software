# SARP-Bot-Software
SPARC Autonomous ROS Platform Robot (SARP).  SPARC's IEEE SoutheastCon Hardware Competition 2019 robot has great hardware for numerous robotics application in a small form factor.  The practice bot will be revamped to support an electronics upgrade to allow more processor intensive computations.  See the [main repository](https://github.com/SPARC-Auburn/SARP-Bot) for more general information.  See the [original repository](https://github.com/SPARC-Auburn/IEEE-SoutheastCon-2019)

## Robot Operating System (ROS)
The need for a simplified approach for localization sparked the switch to using the Robot Operating System (ROS.) ROS greatly simplifies complex localization, mapping, and pathfinding algorithms; all of which are incredibly essential to this competition, although, heed was taken at first because of the vast learning curve coupled with ROS. A very layman's explanation of ROS is that sensor data is input nodes, which house all the vital information for the sensors such as programs, readme files, or launch files. Nodes can talk to each other and request information, process the information, and send to another node. A graphical explanation of how our sensor nodes communicate is shown in Figure 1 below. We have four sensors; LIDAR, Camera, IMU, and encoders that create four ROS nodes. The LIDAR, IMU, and Encoder nodes all input into the localization node, this node coupled with the camera node create what we call the decision maker or essentially a path planner. The decision maker then outputs motor control commands to move the robot.

<img src="https://github.com/SPARC-Auburn/IEEE-SoutheastCon-2019/blob/master/Electrical-Hardware/Diagrams/software-architecture.PNG"  width="400px"/>

**Figure 1:** ROS Node Diagram

Figure 7 shows the resulting ROS graph of our implementation.  After testing, we determined the IMU was not adding any additional helpful information for localization so it was omitted from the final design.

<img src="https://github.com/SPARC-Auburn/IEEE-SoutheastCon-2019/blob/master/Images/Software%20Development/ROS_Graph.png"  width="600px"/>

**Figure 2:** ROS Node Diagram

## LIDAR and Localization

The LIDAR sensor is one of the main components of localization; this sensor acts sort of like a GPS for the robot. The YDLIDAR has a 360-degree scanning range with a 10-meter range. As stated previously and shown in Figure 1 the LIDAR will be a node, and this is one of two inputs into the localization node. ROS greatly simplifies the localization process as it already has the algorithms needed, one being AMCL. Adaptive Monte Carlo Localization is the process of tracking the pose of a robot against a known map, which is our case. AMCL takes in a map, LIDAR scan, and transform messages, and outputs pose estimates. As stated one of the first things we need to implement the AMCL algorithm is a known map, which is the competition field. Our original process of creating a map was creating a jpeg image and feeding it thru a python program launched within ROS to return a map. However, errors were given when adding the LIDAR scan data to the map.  To solve this issue, we had to pre-map the field with actual data using simultaneous location and mapping (SLAM).  This proved to be very effective, especially when there was a large skew in the field shape.  See Figure 3 for a view of the robot’s orientation and position plotted on a map in ROS.

<img src="https://github.com/SPARC-Auburn/IEEE-SoutheastCon-2019/blob/master/Images/Software%20Development/rviz-cropped.png"  width="400px"/>

**Figure 3:** RVIZ map with localized position and orientation

## Rotary Encoder and Odometry
In order to have more precise measurements in movement, we will use two capacitive encoders attached to the motors.  The encoders measure and count the turns of the motors, allowing us to know how far the robot will travel with one motor turn.  Using this alongside the localization and object detection, we would be able to move the robot the exact distance towards any detected object to ensure the robot picks up the object.  This will also prevent the robot from pushing any object into a position that would make it difficult to pick up.

The encoders are wired and read using a Teensy microcontroller.  The Teensy features built in interrupts that allow regular updates from the encoders.  This is required as the encoders measure rotations in ticks, these ticks update at a very fast frequency, requiring processing just as quickly.  The Teensy reads the encoder ticks and calculates the angular acceleration, which is then sent to the Raspberry Pi via serial.  
## Visual Detection
We are using the Raspberry Pi Camera Module to detect the debris objects.  By using the open-source computer vision library, the Pi can extrapolate the debris’s pixel location, pixel size, the angle from center, color, type, and approximate distance from the robot.  It uses this data to make decisions on where the robot should go.  For E-Day, we demonstrated that the robot can turn towards an object, drive until the object is right in front of it and then turn and find a different object.  The program worked really well.  The biggest issue is that the camera was pointed too far up and needed to be mounted at a lower angle.  The next step was integrating visual detection with ROS.  A custom message group was created to pass the object information to the main node for navigation.
## Navigation
Once the robot is capable of finding itself on a map with localization, it becomes necessary to plan the steps from point A to point B. There are several ROS nodes available for this that we attempted to use listed below.

* DWA (Dynamic Window Approach)
    * Smoother paths
    * Requires more computation power
* FTC (Follow the Carrot)
    * More simplistic approach, less computation
    * Does not avoid obstacles well
* Move_Basic
    * Simplest planner
    * Turn to goal, move until goal is reached
    * Not always accurate
    * Does not avoid obstacles

Ultimately, our design settled on Move_Basic, as it provided the simplest approach with parameters that were in our scope of design. Aside from Move_Basic, most path planning algorithms use a form of costmaps. These costmaps determine what path would “cost” the least in terms of time and distance. Most algorithms also use both a local costmap and a global costmap.

The local costmap will calculate where the robot should go in its immediate vicinity, this is usually calculated in real time as the range of the map is shorter and in relation the position of the robot. The global costmap will calculate a path using algorithms such as A*. Using both of these costmaps, the robot can efficiently navigate its space avoiding obstacles it can see.

## Using the System (4/25/19)
The following guide is representative of the system as of the date above. Method of start is likely to change. ALL OF THESE COMMANDS MUST BE RUN IN A LINUX TERMINAL.

The robot is run on ROS, Robotic Operating System. ROS supplies a modular system that does numerous things, but primarily allows different ‘nodes’ to operate on different computers, publishing and subscribing to different topics. In our case, that is two Raspberry Pis. The Pi’s are named d2.local (192.168.1.43) and r2.local (192.168.1.42.). While it is possible to connect to these addresses via VNC viewer, it is recommended to use the ssh command like shown below. You will need two terminal windows for accessing each Pi. (Note: Don’t use the local addresses for connecting to the Pis, use the hostnames d2.local and r2.local)

```ssh ubuntu@r2.local```

This will allow you to access the terminal for the system without bogging it down, as it is typically sensitive to graphical programs being run on it. Note that both Pis must be connected to the same network to use them independently, and AU_WiFi does not work for that. Using a Wi-Fi hotspot is recommended. If only one is able to be connected, ssh into the other via the first one over its local connection (ethernet).

Now that you are connected, you can begin to run the program. All of the ROS launch files are started via shell files, located in ieee-2019-electrical-software/launches on both Pis. 

```cd ~/ieee-2019-electrical-software/launches```

* To launch the required components for the ROS navigation stack, on d2.local, launch ./ftc.sh. 
* To launch AMCL, the move_base node, and the main algorithm node, launch ./amcl.sh on r2.local.

This will boot up the system and begin running whatever is in main_node. If you want to stop the system, it’s recommended to use the kill.py file also located in the /launches file. This way starting new nodes will not overwrite each other and create undesirable bugs.

With the system running, there are several tools ROS provides to allow you to see exactly what is going on in the system. It is recommended to run these on a separate computer or Pi with ROS installed that is connected to the network that the robot is on. This alleviates more computing power from the Pis. To ensure that your ROS computer is getting data from the robot, run the following commands:

```
cd ~/ieee-2019-electrical-software/
source ros.sh
```
* rviz – A visual GUI showing the map, current path, current location, and other useful information
    * By default, if the system is connected properly, RViz will boot up with the map server already loaded in from the roscore you connected to with source ros.sh. Allowing you to see what map is currently loaded in
    * To see other topics not listed by default, on the left pane there is a button that says “Add”, which will allow you to add topics to your view panel. In that window, the “By topic” tab will give you the current topics rviz can see that you can subscribe to. (ie. amcl_pose, current robot position)
* rqt_graph
    * Application gives a visual representation of all the nodes the roscore can see and what topics they are publishing and subscribing to.
* rostopic echo topic
    * A simple terminal output of the topic you choose
* rosnode list node
    * Lists the nodes currently running under the roscore

There are numerous commands that ROS also includes that are a bit too in depth to list here, however more information and tutorials on ROS can be found at http://wiki.ros.org/.

## Debugging (04/25/19)

The following guide is representative of the system as of the date above. Methods are subject to change as the system changes.

### “I can’t connect to either Pi”
* Plug each one in manually to a computer monitor and check that they are connected to the same network you are
* Attempt to ping the Pis via their addresses on your network. This can be checked by using ifconfig

### “RViz, RQT, etc.. can’t connect to my Pi / I can’t see any data”
* Most of the time this happens when one Pi is not connected via WiFi to your network. 
* You’ll find that the nodes run on that Pi will not be seen in the RQT graph.

### “My robot won’t localize itself after moving”
* This is an in-depth problem, as there can be many sources of error. AMCL is what allows the robot to determine where it is in the global space. It uses primarily the laser scan data and the odometry (encoders). Ensure both are outputting what you would expect with rostopic echo
* Ensure you define the correct initial position in the amcl_diff.launch file. Don’t forget the orientation (in radians)

### “The robot won’t go to the correct spot I defined”
* Ensure the position you defined is within the map space using RViz.
* There are currently several bugs with the navigation stack to be worked out. Using the move_base nodes that use costmaps seem to have the most trouble with navigation

## Known Bugs (04/25/19)
* Move_Base node is still sporadic and a primary cause of being unable to navigate the space
* The /pid_velocity nodes are currently untuned, so they have been changed to a static range of 20%-50% in motor power terms. (The robot overcomes the power of inertia at 20%) This should be fixed at some point to allow the robot to be more accurate
* AMCL sometimes will lose its position if it does not move for an extended period


