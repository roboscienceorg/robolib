# Contributing

Welcome to RoboLib, and thank you for considering contributing! If you haven't
checked out our [Code of Conduct](./CODE_OF_CONDUCT.md), please do so before
contributing. We will enforce this Code to all contributors, so please familiarize
yourself to it.

Prior to any code contributions, please reach out to the maintainers to discuss and
flesh out any potential additions to ensure your time is not wasted on development
that may not match the project strategy. The current maintainer can be reached at 
jeff.mcgough@sdsmt.edu, and the parent project's website can be found 
[here](http://www.roboscience.org/). 

## Branch Strategy

There are two main branches for this repository: *master* and *development*. At the
current time, the *master* branch is reserved mainly for releases, while the *development*
branch is the base for all development.

Hotfix branches may be introduced on either *master* or *development* as necessary.

## Pull Requests

To develop a new feature, base a feature branch off of the current state of the development
branch. Once development has been completed and all tests are passing, use the web interface
to create a [pull request](https://help.github.com/articles/about-pull-requests/). Feature
We request that you tag at least one maintainer as a reviewer and assign the request to that
maintainer to ensure your contribution receives swift attention.

Upon successful code review (and potential edits), your feature will be added. Simple!

## Testing

This project uses [Travis-CI](https://travis-ci.org/) for continuous integration. The 
automated testing follows the test plan found [here](./DesignDoc/source/testing.rst), which
utilizes the Python [Nose](https://nose.readthedocs.io/en/latest/) package. 

Tests are placed in the **/tests/** directory, following the naming convention *test_<feature>.py*.
We encourage you to write thorough unit and system tests following the example set by the 
current test files found on the repository.
