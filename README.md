Project Goals
-------------
This is the codebase for Numa 3, copied from my work for Numa 2 in 2019, and extended.

Numa 3's goals for the 2024 competition center around working towards partial autonomy of a mech.
- Get a linux computer talking to the PyBoard - Raspberry Pi 3A+
- Get ROS running on the Pi (Ubuntu 16.04 with ROS Kinetic, via Ubiquity Robotics' image for the RPi3)
- Add the LDLidar 2d laser scanner to the robot
- Write a translation layer to convert ROS /cmd_vel messages to the walk/turn control logic scheme I wrote for the pyboard.
- Navigate on a ROS map using laser-only odometry
- User interface is to point and click and send a pose to the robot, e.g. in RViz, and have it navigate there.
- Minimum data transfer:
  - Send goal pose
  - Receive robot's pose (i.e. laser-based odom)
  - Don't expect to view laser scan in real time during competition.
- Teleop will still work with the Xbee based Arbotix Commander
- Bonus: Because extra payload of laser + RPi, Use stronger servos in key joints
  - Playing with HiWonder HX-35HM servos which are slightly smaller than an AX-12, and about the strength of an MX-28
- And... cleaner better code than where I left off in a mad rush to get things working 5 years ago!

The above is very much a "cool thing to do" and not at all expected to "make the mech more competitive".
It aims to be incremental progress, and we'll see what I do for the next major competition.

There are some definite challenges, such that everything might just not work in a competition environment.
Some that come to mind as of early 2024:
- Wifi communication in the Robogames environment could very totally break the ability to do ROS stuff.
