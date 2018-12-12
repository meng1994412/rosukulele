# rosukulele package
#### Northwestern University
#### ME 495: Embedded Systems in Robotics
#### Chenyang Meng, Levi Todes, Yuwei Xia, and Joshua Zaugg

## Introduction

The goal of this package is to have a RethinkRobotics Sawyer robot autonomously tune a ukulele. The ukulele, a custom pick, and a custom tunig peg are all in designated locations on a platform along with an alvar tag. Sawyer senses the alvar tag and by nowing the relative locations of each of its needed tools, it tunes the ukulele. The main pattern occurs like this:

1. Pick up the pick
2. Pluck the designated ukulele string
3. Listen to the produced pitch and find error from expected pitch
4. Set down the pick
5. Pick up the tuning block
6. Move to the designated tuning peg on the ukulele
7. Turn the peg by an ammount proportional to the pitch error
8. Set down the tuning block
9. Repeat steps 1-8 until pitch error is below a specified tolerance
10. Repeat steps 1-9 until on each of the 4 ukulele strings

The expectation is to have a tuned ukulele at the end of the all of these steps. This entire process can be ran by running the command `roslaunch rosukulele main.launch _ar:=true _setup_only:=false`

## Nodes

[fetch.py](https://github.com/zigzaugg/rosukulele/blob/master/src/fetch.py) - This script starts the 'fetch' node and makes the 'move_to' service available. This node is built off of the `intera_examples` script `go_to_cartesian.py`. Because of this, the service definition `MoveTo` accpets a call in the form of a string and parses arguements in the way the script it's built on used to parse command line arguements. 

[getPitch.py](https://github.com/zigzaugg/rosukulele/blob/master/src/getPitch.py) - This `pitches` node subscribes to the `/pitch` topic published by the `tuner`. It also establishes the service `get_pitch`. When the service is requested, it waits for 3 messages, then takes the pitch average of the next 3 messages and returns them. 

[gripper.py](https://github.com/zigzaugg/rosukulele/blob/master/src/gripper.py) - This `gripper` node is built off of the `intera_examples` script called `gripper_keyboard.py`. It creates the service `grip_pls` which can be called with either `o` to open the grippers or `c` to close the grippers. 

[transform.py](https://github.com/zigzaugg/rosukulele/blob/master/src/transform.py) - This `uku_tf_listener` node creates the `transformation_matrix` service. The service accepts a string that corresponds to x y z coordinates in the frame created by the alvar tag (often reffered to as the `/Ukulele` frame but really called `/ar_marker_2`) and uses `tf.LookupTransform` to convert these coordinates into the base frame. It returns these coordinate in the `/base` frame.

[tuner.py](https://github.com/zigzaugg/rosukulele/blob/master/src/tuner.py) - This `tuner` node is adapted from [python-tuner](https://github.com/mzucker/python-tuner) by Mark Zucker. It uses fourrier transforms on every 0.1s of audio to get the dominant pitch and publishes this on the `/pitch` topic. 
[tune.py](https://github.com/zigzaugg/rosukulele/blob/master/src/tune.py) -

## Messages

[pitch.msg](https://github.com/zigzaugg/rosukulele/blob/master/msg/Pitch.msg) - This message is published by tuner and read by getPitch. It contains a single floa32 called pitch. 

## Service Files

[GetPitch.srv](https://github.com/zigzaugg/rosukulele/blob/master/srv/GetPitch.srv) - This service file defines the `get_pitch` service.

  int8 numSamples
  ---
  float32 pitch

[Grip.srv](https://github.com/zigzaugg/rosukulele/blob/master/srv/Grip.srv) - This service file defines the `grip_pls` service.

  string grip
  ---
  string success

[MoveTo.srv](https://github.com/zigzaugg/rosukulele/blob/master/srv/MoveTo.srv) - This service file defines the `move_to` service.

  string call
  ---
  string response

[Transform.srv](https://github.com/zigzaugg/rosukulele/blob/master/srv/Transform.srv) - This service file defines the `transformation_matrix` service.

  string call
  ---
  string response

## Launch Files

[ar_track.launch](https://github.com/zigzaugg/rosukulele/blob/master/launch/ar_track.launch) - This launch file is from ar_track_alvar. It launches all ar tracking related systems and if it can see the alvar tag, it publishes on the `/ar_track_pose` topic and connects it to the camera frame.

[main.launch](https://github.com/zigzaugg/rosukulele/blob/master/launch/main.launch) - This launch file starts all nodes in this package except for `tune.py` by default. It accpets the arguement `setup_only` that can be set to `false` to start `tune.py` as well. It also accepts the arguement `ar` that can be set to true to also run `ar_track.launch`. 

## STL files

[PegHolder.stl](https://github.com/zigzaugg/rosukulele/blob/master/stl/PegHolder.stl) - This is the stl file (in mm) that is used to 3D print the custom tuner. It is designed to fit with minor clearance around a mitchell concert ukulele peg. 

![Tuning Block in OnShape](https://github.com/zigzaugg/rosukulele/blob/master/Images/TuningBlockSTL.png)








