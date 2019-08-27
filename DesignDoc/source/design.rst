.. role:: math(raw)
   :format: html latex
..

.. role:: raw-latex(raw)
   :format: latex
..

Design and Implementation
=========================

The Robotics library will be developed has several different areas or 
components.


Systems Goals
-------------

As discussed with the stakeholders, the Senior Design team will be delivering 
library code in Python containing all of the algorithms from the Robotics book 
written by Dr. McGough, a professor at South Dakota School of Mines and 
Technology. Thorough documentation and tutorials for every module and setup 
files will be provided so that users with very little to no experience spend no 
time in setting up their environment and getting started with learning the 
concepts.

System Overview and Description
-------------------------------

There are 3 major components to this project, and they are library modules, 
documentation, tutorials and setup files, and map object. All these are 
independent, but make no sense without one another. Setup files help run the library modules and these modules will make no sense to an inexperienced user 
without the documentation and tutorials. The map object removes all dependency 
from Veranda and helps plot the path taken by a robot when it is given various algorithms. 

Major System Component #1
~~~~~~~~~~~~~~~~~~~~~~~~~

Library Source Code: This contains all the call routines in Python which the 
user will be able to use to learn Robotics concepts. The algorithms developed 
will be all those described on the RoboScience website under "Software".


Major System Component #2
~~~~~~~~~~~~~~~~~~~~~~~~~

Examples: This will contain tutorials, heavily documented code and setup files 
which will help the user understand how the modules are used and how they interact with each other and they can also setup thier execution environment. 

Major System Component #3
~~~~~~~~~~~~~~~~~~~~~~~~~

Map Object: This is a very useful data structure which allows to the user to 
visualize the path taken by the robot using matplotlib. 

Technologies Overview
---------------------

Ubuntu 16.04 LTS (Xenial Xerus) operating system was used to develop as well as 
Robotics Operating System 2. Initially Veranda was used as a simualtion 
software, but we decided to move away from it as it was actively being 
developed and the students who were using it kept finding bugs. Besides these, 
various libraries from Python like the SciPy stack and matplotlib were used. 

Development was done using Python and an Object-Oriented approach was taken. 
Python is popular and widely used and has a clean syntax, which means that the 
code is easy to read and write. GitHub was used for version control. 


Architecture and System Design
------------------------------

The structure of the library required the most architecture. Each module
contains Python objects which represent a component of that topic.

For example, in the Motion Algorithms section, you have functions which 
actually run the algorithm, and class objects which provide the infrastructure
such as the environment which the robot operates in.

Save for the algorithm section, each module is built as independent from the 
other modules as possible. This allows the user the flexibility to write code
to tie them together, but they can operate as they are. Since robotics lends
itself to innovative tasks, the design team attempted to couple everything
rather loosely to ensure an appropriate amount of flexibility in use.

Design Selection
~~~~~~~~~~~~~~~~

We initially developed and displayed the robots movements using Veranda and ROS 
2, but since both were actively being developed, they still had 
bugs. Therefore, we decided to move away from it and made our own map object, 
so that we did not have to depend on external libraries which may fail at 
times. 

Each algorithm was written on a different branch and was heavily tested. Only 
after code reviews were they merged into the master branch. 

Data Structures and Algorithms
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The only special data structure we used was the Map Object, which allows a user 
to create a robot, some obstacles and a goal, and it displays all this and the 
path taken by a robot on a graph using matplotlib. Python builtin data 
structures like list, map and arrays were also used.

Communications
~~~~~~~~~~~~~~

The team met twice a week every week in person, once on Tueday with the sponsor 
to update him on our progress, and once on Friday to make objectives and review 
code. Besides that, the team used Discord, a popular online messaging 
application, to decide on meeting times and discuss issues if they could not 
meet. A git runner was set up on discord so that the team got a notification 
every time someone pushed something to the RoboScience repo. 


Major Component #1
-------------------

Since the goal of this project is education, this is probably the most important component in the application. All the source code is present here 
and the user can call it by including the library in their scripts and using 
the various functions inside the classes. The modules can also be divided into 
categories. There are kinematics, ROS dependent and Map Object dependent 
algorithms which help visualize path planning algorithms. 

Technologies Used
~~~~~~~~~~~~~~~~~

The libraries were developed on Ubuntu 16.04 operating system and Python 3. 


Architecture Diagram
~~~~~~~~~~~~~~~~~~~~~

Robots are inidividual classes, and they can be easily called and used by 
including their classes in the file headers. Tutorials and examples are 
provided to use each module.  



Major Component #2
-------------------

Once again, since our main goal was education, we wanted this library to be 
user friendly. Hence, we provided setup files, tutorials and heavily 
documented the classes so that beginners who have no knowledge about robotics 
or the operating system, spends very little time in understanding how to 
install the dependencies and using the libraries. 


Technologies Used
~~~~~~~~~~~~~~~~~

The documentaion was done in the Python files itself. Vim and Gedit were used 
to write them. Latex was used to format the project details and description 
documents. All this is present on the GitHub repository 
https://github.com/roboscienceorg/robolib

Component Overview
~~~~~~~~~~~~~~~~~~

The documentation will include both API references generated from the Python 
Sphinx library, as well as tutorials for each module. This should provide both
beginners and experienced programmers with the tools necessary to get off the
ground quickly.


Major Component #3
-------------------

This is the Map Object which helps move away from Veranda and ROS 2. It is 
basically a data structure which helps the user make a robot, obstacles and a 
goal, and diplays all this using matplotlib. It shows colors which tell user 
how the planning was done and which route was most likely to be taken. 

Technologies Used
~~~~~~~~~~~~~~~~~

Matplotlib and Python 3 were used to develop the Map Object. Numpy was used to 
speed up the calculations. 

Component Overview
~~~~~~~~~~~~~~~~~~

Helps make a robot
Helps make obstacles
Helps make a goal

The user can either do this using matplotlib or hardcoding it at the beginning. 
If the user decides to use the GUI, then he has to click finish to get done 
with editing it.
