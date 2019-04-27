Overview, Description and Deliverables
======================================

RoboScience is a Python library which contains all of the algorithms 
from the book, Robotics, written by Dr. Jeff McGough, a professor at 
South Dakota School of Mines and Technology, Rapid City, South Dakota. 
This is inteded for educational purposes, and allows the user to call 
routines in Python visualize a robot in a simple environment by using 
control panel and a display tool.


Current Version [0.0.1]
------------------------

*Prepared By:*
*Aaron Campbell,*
*Jeremy Goens,*
*Soham Naik*

|  ***Revision History***

========  ============  =======  ================
  Date       Author     Version     Comments
--------  ------------  -------  ----------------
09/28/18  Soham Naik    1        First Draft
04/26/19  Aaron C.      2        Submission Review
========  ============  =======  ================



Team Members and Team Name
--------------------------

RoboScience - 
*Aaron Campbell,*
*Jeremy Goens,*
*Naomi Green,*
*Soham Naik*

Client
------

Our client and sponsor is Dr. Jeff McGough, professor at South Dakota 
School of Mines and Technology. He has founded the RoboScience organization
with the goal of fostering an open source robotics community focused on
education. The developed software will supplement the open-source book found
at http://www.roboscience.org and contribute to the purpose of RoboScience.

Project
-------

This project encompasses an open-source robotics library that will accompany an open-source robotics text book.

The goal is to provide an accessible and easy-to-use Python library for students of the robotics field. If effectively created, this project would enable students from other technical backgrounds who are not well-versed in the minutia of robotics programming to learn through a higher-level abstraction. 

Since the goal is education, simply developing the software will not satisfy the mission. Tutorials and integration into the open-source textbook are necessary. A thorough documentation and a strong repository of examples will be delivered along with the software.

The focus of the library will be the content addressed in the robotics textbook found at http://www.roboscience.org. This library will be structured to encourage further development in areas not covered in the textbook. 

Since the goal is a quick-to-grasp library, this project will be developed in Python, one of the two languages used by the Robot Operating System (ROS). The library will include setup files which enable a user with no prior knowledge of Python to install the software necessary to run the examples.


Mission Statement
~~~~~~~~~~~~~~~~~

The mission statment of this group is to support the education of Robotics through development focused on ease-of-use for the end-developer.

Elevator Pitch
~~~~~~~~~~~~~~

Our product will be a robotics library "for the people." Since robotics is the culmination of many different disciplines, the designers of robots come from varying degree of computer science ability and knowledge, but software has become an imperative to the modern robotics environment.

As such, facilities such as the Robot Operating System (ROS) have been developed to enable easy communication between robotics components. Even still, the learning curve for a non-developer to feel comfortable with ROS is steep and can easily require more experienced personnel to debug software problems. 

The purpose, therefore, of this library is to provide the user with a well-supported and documented development environment which works in conjuction with the open-source textbook being developed. The library will be flexible enough to enable development without ROS, but powerful enough to support ROS without needing the end-user to touch the nitty-gritty.

As a final note, this system will remain an educational tool, and the purpose is to demonstrate key concepts and abilities in the robotics field, not to build a library capable of commercial robotics development.

Purpose of the System
~~~~~~~~~~~~~~~~~~~~~

As described above, the purpose of this system to provide an easy-to-use educational tool which accompanies the freely available robotics textbook found at http://www.roboscience.org. It will provide a hands-on experience for a student of the book, while allowing development beyond the material covered in the text.

Business/Market Need
--------------------

Due to the open-source nature of this project, the business/market need are defined non-traditionally.

As described above, education is the profit that we hope to achieve. By enabling people to move beyond the traditional introductory to robotics tutorials (i.e. Lego Robotics) into a more advanced, yet still nurturing development environment, we hope to expand interest in the robotics field.

Product Description:
    Open-Source Python library that allows for robotics algorithm programming with or without ROS integration.

Key Business Goals:
    Develop a small community of contributors.

    - Increase pull-requests and forks each semester through 2020.
    - Move the maintainence of this library from a group such as the Senior Design Group to a consistent community of experienced robotics students.

Primary Market:
    Students (traditional or otherwise)

Secondary Markets:
    Potentially educators who are looking for resources on robotics education.

Assumptions:
    |   

    -  The user of this library has an introductory knowledge of Python 3.

    -  The user has access to the RoboScience textbook and the RoboLib Design Documentation

Stakeholders:
    |   

    -  RoboScience.com

    -  Developers

    -  Future Students


Deliverables
------------

The deliverables of this project extend beyond the development of a useable 
library. To align with the mission statement of both this project and the 
RoboScience organization, strong documentation and tutorials are required.

This project will produce the following:

    - Library Source Code: This will be the actual library. The majority of the 
    library will be structures which the user can import and use within their 
    own Python scripts. The algorithms developed will be textbook routines 
    that are described on the RoboScience website under "Software". An external 
    map tool will be provided which the user can use to visualize the 
    algorithms. This removes the dependency of 2D simulation softwares like 
    Veranda. 

    - Tutorials: This will be a collection of tutorials which showcase the 
    various objects held in the library and numerous use cases for each object. 
    These will be developed under the supervision of the RoboScience textbook 
    editor to ensure that they are written in a manner consistent with the text. 
    This will be mainly referred to by beginners or users from non-technical 
    backgrounds. 

    - Documentation: This will be a thorough description of the library 
    resources. Separate from tutorials, this will be utilized mainly by more 
    experienced programmers who are looking to understand the inner-workings of 
    the library without delving into the source code.

    - Setup files: These files will provide a simplistic install process for 
    the user. It will install all necessary packages and provide the user with 
    a list of useful instructions for their development and it will even allow 
    them to setup their own environment.
    
    - Veranda: Additional sensors, maps, robots and support routines will be 
    provided for Veranda which the user will be able to use.
    
    - Machine Learning: Artificial Neural Networks for filters, planners, 
    vision and kinematics will be provided. This will help the robot sense the 
    environment in a better way and will help better visualize the algorithms. 



Software
~~~~~~~~

These tools will be built on Ubuntu 16.04 LTS (Xenial Xerus) for the Robotics 
Operating System 2.0 (ROS2).
While not all components of the library will require ROS2, all will require 
Python3 with the SciPy Stack (numpy, scipy, matplotlib, etc.).

The intstallation instructions for ROS2 are provided in the Robotics book as 
well as on the ROS2 repository.

Hardware
~~~~~~~~

A computer which has at least 4 giga bytes of ram and 2 gigabytes of 
free space.

