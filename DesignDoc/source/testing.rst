System and Unit Testing Design
==============================

This section describes the approach taken with regard to system and unit
testing. This chapter does not describe the outcome of those tests. That
will be described in the prototypes chapter.

Overview
--------

For this project, there were two main types of tests: unit and usability.

The unit tests were to ensure that each module provided the necessary
functionality and were useful in ensuring no pull request would "break
the build" by failing any unit tests. The Python *nose* framework was
utilized to implement unit testing on all the modules.



The usability tests were conducted by applying our library as if the 
developers were students in the CSC 415 class, for whom the book was
written. This allowed the developers to make design choices which 
produced a functional library (for a robust yet unusable library
does not encourage further development).


Provides a brief overview of the testing approach, testing frameworks,
and general how testing is/will be done to provide a measure of success
for the system.

Each requirement (user story component) should be tested. A review of
objectives and constraints might be needed here.

Dependencies
------------

The only dependency for testing was the Python *nose* module. This can
be installed via PIP by itself or as part of the SciPy stack, which is
a requirement of this project.

This will be installed if the user follows instructions found in the 
installation ReadMe and utilized the provided setup file.


Unit Test Development and Use
-----------------------------

As mentioned in the dependency, the unit tests were developed by making
use of the Python *nose* package. This package is built atop the *unittest*
module which comes pre-packaged with Python.

The purpose of the framework is to allow ease of development and running of
unit tests by allow the user to simply follow a naming convention. If the 
written tests follow the *nose* guidelines (see 
`here <https://nose.readthedocs.io/en/latest/testing.html>` for more details),
then the developer simply need run::

    nosetests

to run all the tests in the current project. These tests may be limited
to the specific tests by running a command such as::

    nosetests <test_file>

This allows the developer to easily view which tests are passing and which are
failing.

Unit tests were created to test all functionality, from simple tasks such as
whether an object is actually created when its constructor is called to complex,
multi-step kinematics testing. Also, because all tests in the project will be run,
the user can be sure that changes in their module do not break the tests created
in other modules of the library.


Unit Testing and Requirements
-----------------------------

The tests described above directly relate to the requirements in that *nose*
is a component of the SciPy stack, thus reducing necessary dependencies. Along
with meeting the requirements for a consistent calling structure and using
the Python library, these tests were designed to fit into the simple and 
useable philosophy of the RoboScience organization. They are extensible
and quick to run.

Unit tests are also developed for object interaction, such as when a Map object
is used with a motion planning algorithm.

System Testing
--------------

For the useability testing, each module is used to solve common introductory
robotics problems. Luckily for the development team, this project comes equipped
with a robotics book containing such problems. 

This type of testing is important because it allows the developers to better 
switch into the mindset of the user and test usecases which do not arise during
traditional development environments.

Python allows for both REPL and static file interaction. While developing, testing
almost exclusively takes place in the static file interaction side of the house;
however, usability testing has helped reveal internal workings of matplotlib which
were unknown to the developers until forced to use the developed objects in a
different setting.

System Integration Analysis
---------------------------

The various system components are independent, so there is very little system integration
in this project. Note, however, that objects such as the discrete Map object must interact
with other modules, so testing to ensure that interfaces are never broken and correct
data flow exists are done through unit testing.

Risk Analysis
-------------

In terms of catastrophic failure, this project does not directly pose a threat to human life 
or limb due to the educational nature of this project, but the responsibility to ensure
the final product does not misrepresent a topic is extremely important. If a module incorrectly
describes a process, this could potentially affect the ability of the student to employ
correct robotics procedures in the future.

Therefore, the design team maintains a firm commitment to testing and validation of the library
to ensure this responsibility is not pushed aside.

Risk Mitigation
~~~~~~~~~~~~~~~

To mitigate these effects, the team will maintain a rigid pull request policy where code which
violates any of these tests will not be merged until all tests are passing. This policy will be
maintained by the team manually unless a Continuous Integration server is established.
