Unit, Integration, System, And Acceptance Testing Design
========================================================

This section describes the approach taken with regard to testing the RoboScience Library. It will cover the testing procedure and framework; however, the outcome of all tests will be described in the prototypes chapter.

Overview
--------

Unlike a project which creates one cohesive program with multiple features, this library is divided into modules which are distinct in design and use from each other. For that reason, our project has a particular focus on unit and integration testing with a lack of system testing. Acceptance testing, however, will appear as releases begin to rollout.

Dependencies and Frameworks
---------------------------

Being a smaller open source project, there are few dependencies. The major dependencies for testing and running the modules that were developed are Python and the Python nose module. These can be installed via PIP or as part of the SciPy stack, which is a requirement of this project.
 
This will be installed if the user follows instructions found in the installation ReadMe and utilizes the provided setup file. Following these instructions will provide a setup that run in most developed Operating Systems. Testing on systems outside of Linux will be reviewed more to determine if any major system dependencies are required.

All automated testing is built utilizing the Python *nose* framework, which builds atop the pre-packaged *unittest* Python module. This package was selected as it adheres to requirement that the only Python dependency be SciPy (since *nose* is part of SciPy). Nose will search through a directory structure to find all tests which match an internally defined regular expression (see here <https://nose.readthedocs.io/en/latest/testing.html> for more details). 

The simplicity of the calling command ::

        nosetests

allows the tests to be easily added to the repositories Continuous Integration workflow to maintain code integrity. 

For continuous integration, this project utilizes a free-for-open-source Continuous Integration resources called *Travis-CI*. This framework uses Github webhooks to watch the repository for any alterations to the code base. If a new change has been pushed to Github, then a virtual machine is booted and run with the specifications from the repository's *.travis.yml* file. 

This is a powerful tool which allows for multiple operating systems, Python versions, and dependencies to be tested. While our project is built for Ubuntu Xenial 16.04, this solution was chosen such that testing for other systems could be easily added. 

Future testing environments could include technologies such as Docker or Kubernetes, but these are not currently implemented in the testing plan.

As part of the repository, there will be community contribution guides which indicate how future tests are to be written as well as where to place them in the repository. At the moment, the current Senior Design team is responsible for maintaining the testing resources, but a more permanent maintainer for the library will take this role over in the future.

Unit Testing
------------

As is standard practice, unit tests were constructed for each class in the modules to ensure each correctly provided the necessary functionality. They range in complexity from simple instantiation testing to multi-step kinematics testing. 

Following the practice found in other open-source Python libraries, all tests have been placed in a designated tests folder. The required tests for a contribution will be outlined in the library's *Contributors Guide* which will be hosted on either the Github Wiki or the RoboScience website.

As mentioned above, these unit tests are run by Travis-CI on any push to the repository. It is expected that a developer has also run these tests locally prior to a push so that passing of the automated testing is trivial. These tests will be included in the repository, so any user who clones or forks the repository will have the ability to ensure his or her changes pass the previously created unit tests.

Integration Testing
-------------------

Integration tests are used to ensure that relationships between the modules are not broken. While many of the modules are completely independent and require no integration testing, classes such as the Map object are used by other components such as motion planning algorithms. As such, integration tests are used to ensure the interfaces are still intact and that each component can communicate with the other. 

These tests are developed by the contributor and added to the testing plan identically to unit tests. They appear in files named *test_<feature>.py* in the *tests/* directory of the repository. This enables them to be automated by Travis-CI. 

When it comes to approaches for the integration tests, the current strategy follows a more "Big Bang" approach where all the systems are integrated and the execution is viewed as a whole. This approach was chosen because there are at most two components talking to each other, so there is no module tree to build. As the project grows in scope, this will likely need to be re-evaluated.

System Testing
--------------

Since each module is created to perform a handful of tasks, many of the unit tests are designed to test the functionality of miniature systems. If the project were to have a Quality Assurance member, then system testing would take place regularly by adding more of the complex unit tests which the code base is not run against regularly. As such, there is not a separate process for system testing.

Acceptance Testing
------------------

Each release, the project will work closely with students familiar with the CSC415 class, for which this project was developed. They will be asked to use the provided library to solve homework problems from the course and provide feedback as to the usability of the modules. This testing is extremely important as the purpose of this project is to provide students with a library which will aid learning and reduce frustration over the Python-specific minutiae of the necessary building blocks (a Map object, etc.).

This feedback will be used to develop further user stories and alter the development strategy for the next release.

Test Design and Setup
---------------------

Tests for a specific feature are developed by the contributor who adds the feature. These are then reviewed with the feature by other contributors prior to a merge into development and release.

Bug tracking takes place on the repository using Github's bug tracking software. The maintainer of the repository is responsible for setting priority and severity of bugs, as well as the developer responsible for finding a solution. 

The only challenges encountered in the test environment so far was a Python import error which stemmed from the repository structure initially chosen. After restructuring the project to more closely resemble the desired input strategy, any new environment can easily clone the repository, install the library to its Python environment, and utilize the modules as expected.

As the library grows, the supported versions of Python will likely need to grow as well. Tox is a tool used by many Python developers to aid in testing multiple setups and is a technology which should be considered in further development.

System Integration Analysis
---------------------------

As mentioned above, the architecture of the project is such that most modules will operate independent of one another. This means very few major pieces must fit together. 

An example of two pieces which require integration would be any motion planning algorithm and objects which represent the environment, such as the Map object found in the *tools* module. A motion planning algorithm must be able to query whether a space contains an obstacle so that it may avoid it. Therefore, if either the algorithm or the environment object fail, then the overall "system" of motion planning will fail.

There are no single points of failure for the entire library, but aspects of the library can fail on single points. For that reason, integration and pseudo-system testing are automated on the repository.


Risk Analysis
-------------

In terms of catastrophic failure, this project does not directly pose a threat to human life or limb due to the educational nature of this project, but the responsibility to ensure the final product does not misrepresent a topic is extremely important. If a module incorrectly describes a process, this could potentially affect the ability of the student to employ correct robotics procedures in the future.
 
Therefore, the design team maintains a firm commitment to testing and validation of the library to ensure this responsibility is not pushed aside.

Risk Mitigation
---------------

To mitigate these effects, the team will maintain a rigid pull request policy where code which violates any of these tests will not be merged until all tests are passing. This policy is enforced by the continuous integration hooks, but testing against material from the book should be heavily utilized to ensure code does not mislead the user.

