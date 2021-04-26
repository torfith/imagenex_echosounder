# ROS-EchoSounder-driver-Imagenex-852-000-140
ROS driver for the Imagenex Echo Sounder models 852-000-140 and 852-000-141

## Installation

To obtain user permission to access the serial port, set up appropriate udev rules OR add the user to the dialout group: 

```console
$ sudo adduser <user_name> dialout
```

and reboot.

## Usage

```console
$ roslaunch imagenex_echosounder imagenex_echosounder.launch
```
