User Stories, Requirements, and Product Backlog
===============================================

Overview
--------

The purpose of this project is to provide textbook routines and some 
functionalities so that the user can learn robotics as conveniently as 
possible. In order to achieve this, several requirements are specified the 
users. 

User Stories
------------

This section is the result of discussions with the stakeholders with
regard to the actual functional requirements of the software. It is the
user stories that will be used in the work breakdown structure to build
tasks to fill the product backlog for implementation through the
sprints.

User Story #1
~~~~~~~~~~~~~

I would like to import the textbook routines.

User Story #1 Breakdown
^^^^^^^^^^^^^^^^^^^^^^^

This project is done in conjunction with the open source robotics textbook. 
Therefore, the user would like to call the textbook routines so that they 
understand the material. 

User Story #2
~~~~~~~~~~~~~

I would like to use methods with a consistent calling convention. 

User Story #2 Breakdown
^^^^^^^^^^^^^^^^^^^^^^^

Following conventions is always nice as it makes the code readable and makes it 
easier for further development.  

User Story #3
~~~~~~~~~~~~~

I would like to call routines in Python. 

User Story #3 Breakdown
^^^^^^^^^^^^^^^^^^^^^^^

Python is the language of choice due to its simplicity and readability. It is 
also one of the languages used by ROS.

User Story #4
~~~~~~~~~~~~~

I would like to only use SciPy stack and textook library.

User Story #4 Breakdown
^^^^^^^^^^^^^^^^^^^^^^^

Utilizing packages like SciPy benefit from the packages computational efficiency
and simplifies the project dependencies to a small number of packages. 

User Story #5
~~~~~~~~~~~~~

I would like a map object to use for path planning. An external map tool would 
be nice.  

User Story #5 Breakdown
^^^^^^^^^^^^^^^^^^^^^^^

A map object is required to store environment data and provide the infrastructure
necessary for robot motion and path planning. 

User Story #6
~~~~~~~~~~~~~

I would like a display tool(openCV or mathplotlib) for maps and paths. 

User Story #6 Breakdown
^^^^^^^^^^^^^^^^^^^^^^^

This allows the user to visualize what the robot is doing, rather than
attempting to read numerical output. 

User Story #7
~~~~~~~~~~~~~

I would like a remote control tool. 

User Story #7 Breakdown
^^^^^^^^^^^^^^^^^^^^^^^

The user can control and actually test the robot with an external controller 
rather than interacting simply via code.

User Story #8
~~~~~~~~~~~~~

I would like a control panel to control a robot. 

User Story #8 Breakdown
^^^^^^^^^^^^^^^^^^^^^^^

An additional requirement for this project would be the development of a robot
control panel which would provide graphical user interface for an actual robot.
This panel would present metrics from the onboard sensors and allow user 
interaction with the robot. 

Requirements and Design Constraints
-----------------------------------

The user will have to meet certain requirements to smoothly install and run 
the software.

System Requirements
~~~~~~~~~~~~~~~~~~~

For users who do not have Ubuntu 16.04 installed, these are the recommended 
minimum system requirements to install it. 
 - 2 GHz dual core processor
 - 2 GiB RAM (system memory)
 - 25 GB of hard-drive space (or USB stick or external drive)
 - VGA capable of 1024x768 screen resolution
 - Either a CD/DVD drive or USB port for the installer media

The user will require Robotics Operating System 2, and the installation 
instructions are provided on the Roboscience webpage.

Following are some of packages for Ubuntu 16.04 which are required to run the 
robotics software.
 - cycler 0.10.0
 - kiwisolver 1.0.1
 - matplotlib 3.0.0
 - numpy 1.15.2
 - pkg-resources 0.0.0
 - pyparsing 2.2.1
 - python-dateutil 2.7.3
 - six 1.11.0

Network Requirements
~~~~~~~~~~~~~~~~~~~~

A decent internet connection which allows the user to meet the system 
requirements access the repository.

Development Environment Requirements
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We will be using Robotics Operating System (ROS) 2 since it is required to test 
all of our Robot Software. Our language of choice is Python 3.6 because it's one 
of the languages used by the Robotics Operating System and it will be easy for 
the user to learn, and it has some useful packages like SciPy which makes 
calculations faster. We will be developing on Ubuntu 16.04 LTS (Xenial Xerus) 
since ROS is stable here and it is hardware independent. A Python Virtual 
Environment will be used as it allows us to install specific package versions 
which are different from the ones used by other programs on Ubuntu.

Project Management Methodology
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Senior Design Team will be working in 2 week sprints, and will meet twice a 
week, once on Tuesday at 1pm with the sponsor to update him on the progress of 
the project, once on Friday at 1pm to review the code each team member has 
written.

Specifications
--------------

None yet.

Product Backlog
---------------

GitHub, a web-based hosting service for version control, will be used to keep a 
track of the backlog and sprint status. Initially, in the Fall semester, there 
will be 7 sprints and each one will be of 2 weeks. All the members of the 
Senior Design team will be able to make changes to the repository as they want. 

The initial product backlog consists of all the algorithms from the Robotics 
textbook which are yet to be written, and they are: 
 - Parallel Two Link Manipulator
 - Bugs 1, 2, 3 and Tangent Bug
 - Maze Escape
 - Wave Front algorithm
 - Inverse Kinematics DD robot
 - FK and IK Mecanum
 - FK and IK Steered
 - Value to PWM, PWM to value
 - Simulation of sensors (IMUs, encoders)
 - Interface to sensor
 - Triangulation using lasers
 - Hooks to use OpenCV
 - Lane Detection code
 - Traffic sign detector
 - CNN based lane detection
 - PID control family
 - Low and High pass filters
 - Weighted Average
 - Recursive Filter
 - Kalman Filter (Scalar and Vector) 
 - Extended Kalman Filter
 - Unscented Filter and Sigma Point filter
 - Particle Filters
 - Potential Filters
 - Brushfire algorithm

Research or Proof of Concept Results
------------------------------------

We had experimented with a few design structure, but finally stuck to one after 
sprint 2 as it was approved by the sponsor and it allows further development. 

Supporting Material
-------------------

http://www.roboscience.org/
