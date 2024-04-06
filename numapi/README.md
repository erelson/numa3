This folder contains code for the Raspberry Pi or similar single board computer
that will run ROS and talk via serial to the PyBoard.

Notes on files/folders:

- numa_bringup.launch
- numapi_to_pyboard : ROS package with a node for relaying /cmd_vel topic's
  payload to the pyboard over serial
