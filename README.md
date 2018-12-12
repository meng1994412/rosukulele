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

The expectation is to have a tuned ukulele at the end of the all of these steps. 

## Nodes

[fetch.py](https://github.com/zigzaugg/rosukulele/blob/master/src/fetch.py) - 

[getPitch.py](https://github.com/zigzaugg/rosukulele/blob/master/src/getPitch.py) -

[gripper.py](https://github.com/zigzaugg/rosukulele/blob/master/src/gripper.py) - 

[transform.py](https://github.com/zigzaugg/rosukulele/blob/master/src/transform.py) -

[tune.py](https://github.com/zigzaugg/rosukulele/blob/master/src/tune.py) -

[tuner.py](https://github.com/zigzaugg/rosukulele/blob/master/src/tuner.py) -



## Messages

[pitch.msg](https://github.com/zigzaugg/rosukulele/blob/master/msg/Pitch.msg) - This message is published by tuner and read by getPitch. It contains a single floa32 called pitch. 

## Service Files

[GetPitch.srv](https://github.com/zigzaugg/rosukulele/blob/master/srv/GetPitch.srv)

[Grip.srv](https://github.com/zigzaugg/rosukulele/blob/master/srv/Grip.srv)

[MoveTo.srv](https://github.com/zigzaugg/rosukulele/blob/master/srv/MoveTo.srv)

[Transform.srv](https://github.com/zigzaugg/rosukulele/blob/master/srv/Transform.srv)

## Launch Files

[ar_track.launch](https://github.com/zigzaugg/rosukulele/blob/master/launch/ar_track.launch)

[main.launch](https://github.com/zigzaugg/rosukulele/blob/master/launch/main.launch)

## STL files

[PegHolder.stl](https://github.com/zigzaugg/rosukulele/blob/master/stl/PegHolder.stl) - This is the stl file (in mm) that is used to 3D print the custom tuner. It is designed to fit with minor clearance around a mitchell concert ukulele peg. 








