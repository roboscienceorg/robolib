**Basic Tutorial Differential Drive Robot**
=========================================

Preface 
-------
The following tutorials and all other included algorithms are for educational purposes. They are meant to be used in conjunction with the Roboscience library, Website, and Book. The link to the website and book are below.

Links to RoboScience Library
----------------------------
http://www.roboscience.org/

Description
-----------
A Differential Drive Robot (DDR) is a general use beginner robot. It consists of a solid base and two wheels on both sides of the base. These are connected by a axel that is measured from the connection point of the robot. This is to be the center of rotation for the robot. It also has at least one free wheel (a wheel that does not have a drive attached to it) to help balance out the robot. Each of the wheels can move at an independent speed, which drives the robot forwards or may cause it to turn if there is a difference in wheel speeds.

(Include mathmatics in tutorial?)

Usage
--------

In the Robolib library, you will find a simulation of the basic DDR robot. This file will be called ddr.py, which located in the robots subfolder. 

To be able to use any of the library for the DDR robot, you will need to first import ddr.py this will appear as such:
.. figure:: ./pics/ddrpic1.png
   :alt: Differential Drive Robot import .... Differential Drive Robot  [systemdiagram]
   :width: 75.0%

   Differential Drive Robot import  .... Differential Drive Robot  [systemdiagram]


To be abple to initialize a simple DDR robot, after having imported the library, use the following function to simulate a robot. To begin, you will need the two wheel radii, and the axel length. You will call the function as such:

ddr.DDR( <right wheel radius>, <left wheel radius>, <axel length from wheel to center of rotation>, <(optional; default is 0,0,0) starting position of the center of rotation>)

Once a DDR robot has been created, you have a handful of useful functions available to you. To be able to calclate the global position of a robot, given the angular velocity of each of the wheels, as shown.
ddr. calc_global_velocities(<angular velocity of the right wheel>, <angular velocity of the left wheel>)
(insert ddrpic2)
.. figure:: ./pics/ddrpic1.png
   :alt: Differential Drive Robot import .... Differential Drive Robot  [systemdiagram]
   :width: 75.0%

   Differential Drive Robot import  .... Differential Drive Robot  [systemdiagram]

This function will return the x directional velocity, the y directional velocity, and the global angular veloctiy of the robot.
(insert ddrpic3)
.. figure:: ./pics/ddrpic1.png
   :alt: Differential Drive Robot import .... Differential Drive Robot  [systemdiagram]
   :width: 75.0%

   Differential Drive Robot import  .... Differential Drive Robot  [systemdiagram]

To be able to calculate the Forward Kinematics of you given robot for a given list of velocities, you need a list of the angular velocities of each wheel, along with the list of times you would like to know the velocity of the DDR Robot.

ddr.FK(<list of angular velocities for the right wheel>, <list of angular velocities for the left wheel>,<time interval to provide calculations at>)
(insert ddrpic4)
.. figure:: ./pics/ddrpic1.png
   :alt: Differential Drive Robot import .... Differential Drive Robot  [systemdiagram]
   :width: 75.0%

   Differential Drive Robot import  .... Differential Drive Robot  [systemdiagram]

returns: pos_list – position list
(insert ddrpic5)
.. figure:: ./pics/ddrpic1.png
   :alt: Differential Drive Robot import .... Differential Drive Robot  [systemdiagram]
   :width: 75.0%

   Differential Drive Robot import  .... Differential Drive Robot  [systemdiagram]

The final function for the DDR robot is the Inverse Kinematics. This will calculate an x and y velocity having been provided a list of x velocities, y velocities, x acceleration, and y acceleration.
def IK(self, x_vel, y_vel, x_accel, y_accel)
ddr.IK(<list of x velocities>, <list of y velocities>, <list of x acceleration>, <list of y acceleration>)
(insert ddrpic6)
.. figure:: ./pics/ddrpic1.png
   :alt: Differential Drive Robot import .... Differential Drive Robot  [systemdiagram]
   :width: 75.0%

   Differential Drive Robot import  .... Differential Drive Robot  [systemdiagram]

returns: tupple – left wheel velocity, right wheel velocity at each timestep
(insert ddrpic7)
.. figure:: ./pics/ddrpic1.png
   :alt: Differential Drive Robot import .... Differential Drive Robot  [systemdiagram]
   :width: 75.0%

   Differential Drive Robot import  .... Differential Drive Robot  [systemdiagram]

Example
Include calculations
Show examples

