The Tortoisebot
================

Our friendly little cleaning robot.

Hardware
---------

The tortoisebot is modified vacuum robot with some goodies to make it
fun to use. 

It has the following sensors:

* 3 front bumpers
* Kinect
* IR sensor

If you want to use the IR sensors, there are IR emitters in the lab
which are made for the robot. You can use these to create invisible
walls or to make a homebase for the robot.

Using
-----

Marc has constructed a number of launch files to make using the
tortoisebot easier. Below, I will give a quick sumary of what they
various launch files do.

* `tortoise.launch`: This file launches both the turtlebot driver and
  the Kinect as well as creating a view window to see the output of
  the Kinect. This is the file you normaly want to run.

* `driver.lanch`: Starts the node which communicates to the
  tortoisebot.

* `kinect.launch`: Starts the Kinect and sets it up for use with
  localization and people detection.

* `joystick_teleop.launch`: Runs joystick teleoperation.

* `keyboard_teleop.launch`: Runs keyboard teleoperation.

