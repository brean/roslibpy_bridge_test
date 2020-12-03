# roslibpy_bridge_test

Example project to connect two ros instances using roslibpy to advertise and call a service.
See https://github.com/gramaziokohler/roslibpy/issues/68

This project exists of 3 docker container:

1. **rosservice** a ros installation on the cloud providing a service
1. **robot** is a simulation of a robot, for now it just consists of a ros-node subscribing to the service defined in rosservice
1. **bridge** a python 3 installation advertising the service from the rosservice to the robot and calling the service when the robot tries to access it.
