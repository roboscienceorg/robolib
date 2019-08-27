Release – Setup – Deployment
============================

Setup Information
-----------------

In terms of deployment, our method of deployment is PyPi, a standard repository 
for Python packages.

This allows us to provide a comfortable and simple installation process. The 
only command that a user will need to install the library is:

```bash
pip install robolib
```

NOTE: This currently is undergoing testing, meaning that the library can
be installed from the Test PyPi repository using the command found [here](https://packaging.python.org/guides/using-testpypi/).

This will be removed once the development team moves the release to the 
standard PyPi repository.


Deployment Information and Dependencies
---------------------------------------

The dependencies are maintained as part of the project. Once testing has
concluded with PyPi, these should automatically be installed alongside
**robolib**. Currently, downloading 'requirements.txt' from the [Github](https://github.com/roboscienceorg/robolib)
and running:

```bash
pip install -r requirements.txt
```

will install all of the dependencies.


System Versioning Information
-----------------------------

The current version system is X.X.X

Where the first X marks a major release with no guarantee of backwards
compatibility, the second X marks a release with guaranteed backwards
compatibility within a major release, and the third X is for bug fixes.
