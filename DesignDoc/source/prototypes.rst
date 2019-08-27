Sprint Results and Prototypes
=============================

This chapter is for recording the results of each sprint and documenting
the evolving product. It is a historical record of what you accomplished
in 464/465. This should be organized according to Sprints. It should
have the basic description of the sprint deliverable and what was
accomplished. Screen shots, photos, captures from video, etc should be
used. Expect this to be a long chapter.

Sprint 1 Report
---------------
September 21st - October 4th

The basics of the Sprint were to get ourselves familiarized with the product that
we would be working on. The most efficient plan we came up with was to get the 
ancillary programs installed and to make sure that we all had a stable Linux
machine set up that we could use through the semester. While a slow start, it 
gave us a little more time to read through what we would be working on, as well
as to meet together with our Client and work out an acceptable plan.

Sprint Backlog
~~~~~~~~~~~~~~
 - Two Link Manipulator
 - Parallel Two link Manipulator
 - DD Robot
 - Publisher Example
 - Subscriber Example
 - Basic Motion Algorithm
 - Wave Front Algorithm
 - Mecanum Robot
 - Steered Robot
 - Parallel Two Link Manipulator
 - Bugs 1, 2, 3 and Tangent Bug
 - Maze escape
 - Wave front algorithm
 - Inverse Kinematics DD robot
 - FK and IK Mecanum
 - FK and IK Steered
 - Value to PWM, PWM to value 
 - Simulation of sensors (IMUs, encoders)
 - Interface to sensor 
 - Triangulation using lasers
 - Hooks to use OpenCV
 - Lane detection code
 - Traffic sign detector 
 - CNN based lane detection
 - PID control family
 - Low and high pass filters
 - Weighted Average
 - Recursive Filter
 - Kalman Filter (Scalar and Vector)
 - Extended Kalman Filter
 - Unscented Filter and Sigma point filter
 - Particle filters
 - Potential function
 - Brushfire algorithm

Deliverable
~~~~~~~~~~~
 - Install ROS2
 - Install Latex/Sphinx
 - Contract Completed

Successes and Failures
~~~~~~~~~~~~~~~~~~~~~~
The major successes with this sprint was getting the programs we would be
running for more of the semester up and running, as well as integrating as a 
team for when and how projects would be worked on.

Modifications required (product backlog, design, requirements, etc)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
While there was no major modifications to the project, this is where the initial 
architecture and design for the team started.

Sprint Review
~~~~~~~~~~~~~
For the defined requirements for the sprint, the team filled them well. The programs
were installed quickly in the first week, and the book was reviewed and talked about
at the second meeting we had during that week.

Sprint Retrospective
~~~~~~~~~~~~~~~~~~~~
A major fallback as viewed by the team for this sprint was that we did not
accomplish as much as we could in review. It was a light sprint, and it seemed
as though we should have been able to do more, as it felt like we had fallen behind 
other teams at times.

As for the team dynamics, we seemed to settle into rolls fairly quickly and flawlessly.
We had a person who was well versed in public speaking, one who was versed in translations
between different languages, and one who was prepared to have the semester organized for
the rest of the group.


Sprint 2 Report
---------------
October 5th - October 18th

This was the first sprint in which we dug into the meat of the project and 
hashed out what sort of class structure we were looking for. We took the time
to review the book for the next couple weeks examples, as well as meeting with 
our client to get their views on what the expected out of the project.

This being the first major sprint took us a little bit longer than expected for
having such a light amount of structure; This was in part to working out a time 
when we could all meet as well as learning how to utilize the tools that ROS2
was able to grant us, and the steps that were needed to integrate the classes
that had been completed into the project.

Sprint Backlog
~~~~~~~~~~~~~~
 - Publisher Example
 - Subscriber Example
 - Basic Motion Algorithm
 - Wave Front Algorithm
 - Mecanum Robot
 - Steered Robot
 - Bugs 1, 2, 3 and Tangent Bug
 - Maze escape
 - Wave front algorithm
 - Inverse Kinematics DD robot
 - FK and IK Mecanum
 - FK and IK Steered
 - Value to PWM, PWM to value 
 - Simulation of sensors (IMUs, encoders)
 - Interface to sensor 
 - Triangulation using lasers
 - Hooks to use OpenCV
 - Lane detection code
 - Traffic sign detector 
 - CNN based lane detection
 - PID control family
 - Low and high pass filters
 - Weighted Average
 - Recursive Filter
 - Kalman Filter (Scalar and Vector)
 - Extended Kalman Filter
 - Unscented Filter and Sigma point filter
 - Particle filters
 - Potential function
 - Brushfire algorithm

Deliverable
~~~~~~~~~~~
 - Two Link Manipulator
 - Parallel Two link Manipulator
 - DD Robot

Results of testing
~~~~~~~~~~~~~~~~~~
Direct results of Sprint 2 were the development of the Differential Drive 
Robot Kinematics, Parallel Two Link Manipulator Kinematics, and Two Link 
Manipulator Kinematics.

Successes and Failures
~~~~~~~~~~~~~~~~~~~~~~
A rather productive sprint at the time, the basic examples for what Robotics
basic ideas covered were created. This gave a basic idea for class structure
and solidified the structure for classes.


Modifications required (product backlog, design, requirements, etc)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
No modifications to future Sprints, Design, or Architecture were required this sprint.

Sprint Review
~~~~~~~~~~~~~
Having all modules from this sprint be completed was a good start to the work
that needed to be done. It also gave us the questions that needed to be asked
for future class iterations.

Sprint Retrospective
~~~~~~~~~~~~~~~~~~~~
In review, this sprint seemed to be a little short on what we should have attempted to
complete. However, it also gave us a good idea of how much work each individual module
would require for the rest of the project.


Sprint 3 Report
---------------
October 18th - November 1st

Sprint three was a bit of a larger bite when compared to the first two reports. This
was in part to the want/need to complete some examples on how different integration
with ROS2 and with the examples of how ROS2 operated, as shown in the Publisher/Subscriber
example.

This sprint was more about getting out individual examples to help the layman gain an
understanding of how Robotics worked. It was also about creating simple examples that 
people could follow to learn the basic of Python, ROS2, and Robotics.


Sprint Backlog
~~~~~~~~~~~~~~
 - Bugs 1, 2, 3 and Tangent Bug
 - Maze escape
 - Wave front algorithm
 - FK and IK Steered
 - Value to PWM, PWM to value 
 - Simulation of sensors (IMUs, encoders)
 - Interface to sensor 
 - Triangulation using lasers
 - Hooks to use OpenCV
 - Lane detection code
 - Traffic sign detector 
 - CNN based lane detection
 - PID control family
 - Low and high pass filters
 - Weighted Average
 - Recursive Filter
 - Kalman Filter (Scalar and Vector)
 - Extended Kalman Filter
 - Unscented Filter and Sigma point filter
 - Particle filters
 - Potential function
 - Brushfire algorithm

Deliverable
~~~~~~~~~~~
 - Publisher Example
 - Subscriber Example
 - Basic Motion Algorithm
 - Wave Front Algorithm
 - Mecanum Robot
 - Steered Robot

Results of testing
~~~~~~~~~~~~~~~~~~
Results of Sprint 3 were the creation of several example modules that could be used as
teaching examples.

.. figure:: ./sprint3_test.png
   :alt: Sprint 3 Test Result .... Sprint 3 Result [sprint3result]
   :width: 75.0%

   Sprint 3 Test Result .... Sprint 3 Result [sprint3result]

Successes and Failures
~~~~~~~~~~~~~~~~~~~~~~
This was again a fairly successful sprint, completing most of the modules that could be
used as starter examples to begin learning how individual components work, as well as 
fleshing out the added features that we were looking at adding on later.

The major failure for this sprint was an incomplete module, which was the Steered Robot.
The reason for this was the movement away from the example based modules, and a movement 
towards more diverse sets of code as each individual tended to focus on one or two areas.

Modifications required (product backlog, design, requirements, etc)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
At this sprint, given that we didn't have much background in using Veranda, we altered the 
future sprints to place the Kinematic designs further up the list of what could be completed,
and pushed the failure examples (Bug 1, 2, 3 and Tangent Bug), much further in the project
as we would need a firm understand of how Veranda operated. As well as needing more classes 
designed to be able to integrate and use them given the scope of the project to use them as
examples.

Sprint Review
~~~~~~~~~~~~~
During this sprint, we ran into the issue of not having a large enough understanding of the scope
of the project, that we did not complete one of he modules we were working on and hoping to finish.
While this modules is an essential part of the project, it would not set the project back, as it 
could not be used in conjunction with any of he current modules that had been completed.

Despite the uncompleted modules, this sprint was productive, completing a good portion of what will
end up being the example modules, and the basic modules required for more of the complex design in
later sprints.


Sprint Retrospective
~~~~~~~~~~~~~~~~~~~~
While this sprint was the first one we did not complete a module, it was still a good sprint in 
retrospect. This is mostly because it gave us a large chuck of code that was usable with future
modules that could be used together.


Sprint 4 Report
---------------
November 2nd - November 15th

This sprint shifted developmental gears again as we moved away from the robotics
design and moved towards the develoment of additional modules that would be used
in addition to currently developed classes and module examples.

This modules would be used in conjunction with ROS2 and Veranda to accurately use
a given robotics module previously developed, to use in other example modules.
This is the first sprint that would be developing major components that would be
required to use ROS2 with little technical knowledge that was a major goal of this
project.

Sprint Backlog
~~~~~~~~~~~~~~
 - Bugs 1, 2, 3 and Tangent Bug
 - Maze escape
 - Wave front algorithm
 - FK and IK Steered
 - Simulation of sensors (IMUs, encoders)
 - Lane detection code
 - Traffic sign detector 
 - CNN based lane detection
 - PID control family
 - Low and high pass filters
 - Weighted Average
 - Recursive Filter
 - Kalman Filter (Scalar and Vector)
 - Extended Kalman Filter
 - Unscented Filter and Sigma point filter
 - Particle filters
 - Potential function
 - Brushfire algorithm

Deliverable
~~~~~~~~~~~
 - Value to PWM
 - PWM to Value
 - Simulation of Sensors
 - Interface to Sensor
 - Triangulation using Lasers
 - Hooks to Use Open CV

Results of testing
~~~~~~~~~~~~~~~~~~
Results of Sprint 4 were the creation of several example modules that could be used as
teaching examples.

Successes and Failures
~~~~~~~~~~~~~~~~~~~~~~
There were several failures in this sprint, as we have several modules that
were left partially completed as we moved away from the development on ROS2
and Veranda and moved towards a pure python development for the next several 
sprints.

Modifications required (product backlog, design, requirements, etc)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
This sprint required a major redesign partway through the sprint that set
back the sprint, and future sprints. The major change in this sprint was
to move away from development on ROS2 and Veranda, to working on a python 
based mapping system for examples modules.

Sprint Review
~~~~~~~~~~~~~
This sprint was a bit of a speed bump for the development for the modules that we
had planned to complete. It also gave us a breath of fresh air in that we were not
required to use ROS2 and Veranda, but we were required to have a new mapping object
that we had to develop. This stopped development of several of the current modules,
and adjusted many future modules that were set for the next two sprints.

Sprint Retrospective
~~~~~~~~~~~~~~~~~~~~
This sprint was a major developmental problem for the team as a whole. As we were
moving away from ROS2 based design, we, again, had a lack of basis for development. 
As soon as we had a Map object that was designed, we started on a different schedule
for what we needed to be completed to have a good base by the end of the next couple
sprints.


Sprint 5 Report
---------------
November 16th - November 29th

This sprint places emphasis on developing modules to assist in the learning side
of this project. As well as finishing up modules that had not been completed during
several other sprints. Emphasis is put on modules that would be required for future
develomental modules.

Sprint Backlog
~~~~~~~~~~~~~~
 - FK and IK Steered
 - Simulation of sensors (IMUs, encoders)
 - Lane detection code
 - Traffic sign detector 
 - CNN based lane detection
 - PID control family
 - Low and high pass filters
 - Weighted Average
 - Recursive Filter
 - Kalman Filter (Scalar and Vector)
 - Extended Kalman Filter
 - Unscented Filter and Sigma point filter
 - Particle filters
 - Potential function

Deliverable
~~~~~~~~~~~
 - Map Object
 - Bugs 1, 2, 3
 - Tangent Bug
 - Maze escape
 - Wave front algorithm
 - Brushfire algorithm
 - Unit Tests

Results of testing
~~~~~~~~~~~~~~~~~~
Results of Sprint 5 were the creation of the Mapping Object as a replacement for ROS2 and
Veranda. This created a test figure that could be used in conjunction with the planning 
algorithms and submodules, such as the wavefront algorithm.

.. figure:: ./sprint5_test.png
   :alt: Sprint 5 Test Result .... Sprint 5 Result [sprint5result]
   :width: 75.0%

   Sprint 5 Test Result .... Sprint 5 Result [sprint5result]

Successes and Failures
~~~~~~~~~~~~~~~~~~~~~~
This sprint, contained a number of failures in the modules that we attempted to complete.
Many of the example programs that we were looking at completing were left incomplete and
partially done. This leaves a large hole in the eduational side of the project which
we were looking to fill.

Modifications required (product backlog, design, requirements, etc)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
No modifications to future Sprints, Design, or Architecture were required this sprint.

Sprint Review
~~~~~~~~~~~~~
This sprint of a bit of a large failure on our parts as completion of most of the modules
were not completed. We had a large push on Unit Tests that were added, as well as more
rigorous testing to existing modules, which felt like an accomplishment. However, it
will lead as an example for how not to run sprints for future work.

Sprint Retrospective
~~~~~~~~~~~~~~~~~~~~
This sprint was an example of a lack of leadership in part, and direction, as we failed to
complete most of the modules that we were looking to finish for review, and education.
While this may also point to taking on to many modules to develop at a time, and underestimating
the time required, we also needed to review the amount of time we put into individual modules in
future sprints.


Sprint 6 Report
---------------
November 30th - December 21st

This sprint is a bit of a develomental backlog. With the changes that happened during
the last two sprints, we are looking at finishing the basic modules for our new Map
Object. This as well as some of the basic bugs would be a good start to examples
that can be used as teaching tools.

This sprint may look at being slightly longer than the rest as it does extend into the
Christmas holiday. This will give us a little bit more time to develop some more Unit
Testing using Nosetests.

With the multiple iterations of design that we have developed to the project, finishing
these modules is a much larger priority than completing new modules.

Sprint Backlog
~~~~~~~~~~~~~~
 - PID control family
 - Low and high pass filters
 - Weighted Average
 - Recursive Filter
 - Kalman Filter (Scalar and Vector)
 - Extended Kalman Filter
 - Unscented Filter and Sigma point filter
 - Particle filters
 - Potential function

Deliverable
~~~~~~~~~~~
 - FK and IK Steered
 - Bugs 1, 2, 3
 - Tangent Bug
 - Maze escape
 - Brushfire algorithm

Results of testing
~~~~~~~~~~~~~~~~~~
Results of Sprint 6 were the creation of several example modules that could be used as
teaching examples.

Successes and Failures
~~~~~~~~~~~~~~~~~~~~~~
This sprint had us start back up on the backlog, get things properly set up and added together
to continue to work on the modules that are the main section of the project. This was a good 
start back on this project leading into the end of he semester and into winter break.

Modifications required (product backlog, design, requirements, etc)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
No modifications to future Sprints, Design, or Architecture were required this sprint.

Sprint Review
~~~~~~~~~~~~~
This was a good sprint to end the semester on. Lots of the project base was completed, while leaving
a good portion of the project open to be finished during the next semester.

Sprint Retrospective
~~~~~~~~~~~~~~~~~~~~
A good final sprint. A large change from the original plan at the beginning of the semster. This left a 
good portion of the project to be finished at the begining of the next semester, with any additional 
portions of the project that we are looking at taking on.


Sprint 7 Report
---------------
December 22nd - January 25th

A bit of a longer sprint. This was over the Chistmas break and used to keep our minds focused on the
project. With it being such a long break, we made sure to keep contact over Discord so that we could 
continue making progress.

Most of this Sprint was spent finishing up testing problems for the functions that we had that already 
existed, as well as looking at diffrent ways of distributing this program. Since this is an open source
project, the first meeting back was used to decide on a couple of different licenses we could use.

Sprint Backlog
~~~~~~~~~~~~~~
 - PID control family
 - Low and high pass filters
 - Weighted Average
 - Recursive Filter
 - Kalman Filter (Scalar and Vector)
 - Extended Kalman Filter
 - Unscented Filter and Sigma point filter
 - Particle filters
 - Potential function

Deliverable
~~~~~~~~~~~
 - Research Open Source Projects
	 - Licenses
	 - Contributing Files
	 - Templates

Results of testing
~~~~~~~~~~~~~~~~~~
No outside testing for new modules were completed during this Sprint.

Successes and Failures
~~~~~~~~~~~~~~~~~~~~~~
This Sprint brought about the creation of the licenses that we would use for our first
distribution. As such, while no new modules were made, much more thought was present in 
this sprint than in several others.

Modifications required (product backlog, design, requirements, etc)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
No modifications to future Sprints, Design, or Architecture were required this sprint.

Sprint Review
~~~~~~~~~~~~~
This Sprint was a bit of a letdown compared to what we were hoping to have accomplished over the longer
break that we had. Most of what we had completed was more side stuff than the main structure of our 
project. This lead to a couple setback on the original timeline.

Sprint Retrospective
~~~~~~~~~~~~~~~~~~~~
This was a Sprint that was needed at the time, and looking back, helped set up the rest of the semester
for getting what we finished completed at the time.


Sprint 8 Report
---------------
January 26th - February 9th

This sprint got us back on track for making the entire library that we were hoping to finish. It started
with an initial release for the project, in beta, and the error correction that came from an actual release.

We also received at this time an extra person to help with the build moving foward.

Sprint Backlog
~~~~~~~~~~~~~~
 - PID control family
 - Low and high pass filters
 - Weighted Average
 - Recursive Filter
 - Kalman Filter (Scalar and Vector)
 - Extended Kalman Filter
 - Unscented Filter and Sigma point filter
 - Particle filters
 - Potential function

Deliverable
~~~~~~~~~~~
 - First release
 - Documents 
	 - Readme
	 - License
	 - Code of Conduct
	 - Contributing

Results of testing
~~~~~~~~~~~~~~~~~~
Testing on the first released showed promise, but installation and use with the project was problematic.
The library did not wish to load in correctly and as such, library names do not work correctly at this time.

Successes and Failures
~~~~~~~~~~~~~~~~~~~~~~
We completed a first release! This is a large part of excitement for the group. 

Modifications required (product backlog, design, requirements, etc)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The release actually required a large change in the structure of the project, as the library would not
properly load when called. 

Sprint Review
~~~~~~~~~~~~~
This sprint gave a huge moral boost to the group as we had something that others could download and look
at if they were also interested in robotics, or if they were looking for supplementary material.

Sprint Retrospective
~~~~~~~~~~~~~~~~~~~~


Sprint 9 Report
---------------
February 10th - February 23rd

This was the first sprint where we concentrated on teaching someone else what our project was about. Since
we had gained a new person, we needed to bring them up to speed on what we had been working on and how it
was all meant to work together. 

This also gave us the option of splitting the group up into two subgroups that would allow us to better 
test the Artificial Intelligence section that we were hoping to get to as a group.

Sprint Backlog
~~~~~~~~~~~~~~
 - PID control family
 - Low and high pass filters
 - Weighted Average
 - Recursive Filter
 - Kalman Filter (Scalar and Vector)
 - Extended Kalman Filter
 - Unscented Filter and Sigma point filter
 - Particle filters
 - Potential function

Deliverable
~~~~~~~~~~~
 - Project split into two parts
	 - Library
	 - Artificial Intelligence
 - Prepping for next release


Results of testing
~~~~~~~~~~~~~~~~~~
No testing on modules was completed during this sprint.

Successes and Failures
~~~~~~~~~~~~~~~~~~~~~~
We had a start on the Artificial Network for learning if they could replace Filtering and/or if 
they could be a substitute for either of them.

Modifications required (product backlog, design, requirements, etc)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Since this project was split, it required a split split of the demands on each person.

Sprint Review
~~~~~~~~~~~~~
This Sprint was different from what we had had before as it essentially took our one group 
and split it into two different groups giving us each a different goal for the same project.

Sprint Retrospective
~~~~~~~~~~~~~~~~~~~~
This may have been much later in the semster than we were hoping to get started on this part of the
project, but since we had gained a new teammate, it was a good time for it as well.


Sprint 10 Report
----------------
February 24th - March 16th

This sprint was a much slower sprint due to the decrease in module completion. This did however
start off with the discussion of how we should set the project up to be picked up after we had
completed the given project.

There was also significant work done on the Artificial Intelligence side of the project as well
considering that there was a Paper that needed to be completed soon after the deadline for this
Sprint, along with the required testing and prep needed in the creation of the Neural Net.

Sprint Backlog
~~~~~~~~~~~~~~
 - PID control family
 - Low and high pass filters
 - Weighted Average
 - Recursive Filter
 - Kalman Filter (Scalar and Vector)
 - Extended Kalman Filter
 - Unscented Filter and Sigma point filter
 - Particle filters
 - Potential function

Deliverable
~~~~~~~~~~~
 - Prepping for next release

Results of testing
~~~~~~~~~~~~~~~~~~
No testing on new modules were completed at this time. The modules that were already available
did however see several more test cases created.

Successes and Failures
~~~~~~~~~~~~~~~~~~~~~~
The creation of a basic Neural Network to test against Filtering was a large success of this 
Sprint. The paper that was underway and the different forms of testing were to be seen.

Modifications required (product backlog, design, requirements, etc)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
No modifications to future Sprints, Design, or Architecture were required this sprint.

Sprint Review
~~~~~~~~~~~~~
This Sprint was a bit of a slowdown on the project as a whole. While work was being done on the 
library, a large section of time was dedicated to bringin our new member up to speed, having them 
test out the given modules, and the work on the Neural Network.

Sprint Retrospective
~~~~~~~~~~~~~~~~~~~~


Sprint 11 Report
----------------
March 17th - March 30th

This sprint brough us to the end of the paper, and to the endge of building the filters for the
library. With the completion of he Sensor module, testing against different filters and the fusion
of sensors would be available.

Sprint Backlog
~~~~~~~~~~~~~~
 - PID control family
 - Low and high pass filters
 - Weighted Average
 - Recursive Filter
 - Kalman Filter (Scalar and Vector)
 - Extended Kalman Filter
 - Unscented Filter and Sigma point filter
 - Particle filters
 - Potential function

Deliverable
~~~~~~~~~~~
 - Prepping for next release
 - Simulation of sensors (IMUs, encoders)

Results of testing
~~~~~~~~~~~~~~~~~~
No testing on new modules were completed at this time.

Successes and Failures
~~~~~~~~~~~~~~~~~~~~~~
The simulation of Sensors that was created was far below standard for what woud be needed
and needed more testing

Modifications required (product backlog, design, requirements, etc)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
No modifications to future Sprints, Design, or Architecture were required this sprint.

Sprint Review
~~~~~~~~~~~~~

Sprint Retrospective
~~~~~~~~~~~~~~~~~~~~


Sprint 12 Report
----------------
March 31st - April 13th

Sprint Backlog
~~~~~~~~~~~~~~
 - PID control family
 - Low and high pass filters
 - Weighted Average
 - Recursive Filter
 - Kalman Filter (Scalar and Vector)
 - Extended Kalman Filter
 - Unscented Filter and Sigma point filter
 - Particle filters
 - Potential function

Deliverable
~~~~~~~~~~~
 - Simulation of sensors (IMUs, encoders)


Results of testing
~~~~~~~~~~~~~~~~~~

Successes and Failures
~~~~~~~~~~~~~~~~~~~~~~

Modifications required (product backlog, design, requirements, etc)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
No modifications to future Sprints, Design, or Architecture were required this sprint.

Sprint Review
~~~~~~~~~~~~~

Sprint Retrospective
~~~~~~~~~~~~~~~~~~~~

Sprint Analytics
~~~~~~~~~~~~~~~~

Place your burndown charts, team velocity information, etc here if they
are not discussed above.

Sprint 13 Report
----------------
April 14th - April 16th

Include your report here.  You may have sent the report to your client,
it should be duplicated here.   There is not a separate report submission
for the class.

Sprint Backlog
~~~~~~~~~~~~~~
 - PID control family
 - Low and high pass filters
 - Weighted Average
 - Recursive Filter
 - Unscented Filter and Sigma point filter
 - Particle filters
 - Potential function

Deliverable
~~~~~~~~~~~
 - Kalman Filter (Scalar and Vector)
 - Extended Kalman Filter

Results of testing
~~~~~~~~~~~~~~~~~~

Successes and Failures
~~~~~~~~~~~~~~~~~~~~~~

Modifications required (product backlog, design, requirements, etc)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
No modifications to future Sprints, Design, or Architecture were required this sprint.

Sprint Review
~~~~~~~~~~~~~

Sprint Retrospective
~~~~~~~~~~~~~~~~~~~~


