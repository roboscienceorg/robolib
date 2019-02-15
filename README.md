# robolib - An Educational Robotics Library

Welcome! This repository is the home for the library supporting the 
[RoboScience](http://www.roboscience.org/ "RoboScience Homepage") 
open-source robotics community. The purpose of this library is to support
education for both users and developers in the field of robotics.

## Installation

RoboLib has been tested on Ubuntu 16.04 (Xenial) with Python 3.5. While our installation
process will eventually include a setup script to aid with this process, our initial release
will require a *small* manual process. If you do not have Python installed on your system,
download and install it from [here](https://www.python.org/downloads/ "Python Downloads").

Next, install pip:

```bash
sudo apt-get install python3-pip
```

This will install the [Pip](https://pypi.org/project/pip/ "Pip Homepage") package manager
for Python packages. This will be used in install the RoboLib library as well as dependencies.

Next, download the current RoboLib [release](/releases/latest). This is a .whl file which can
be installed via Pip:

```bash
pip install <release_name>.whl
```

To install the dependencies, either copy or download the
[requirements file](./requirements.txt). Then use pip to find each package:

```bash
pip install -r requirements.txt
```

This will install all the necessary packages to run the modules within RoboLib.

You are now ready to follow our [tutorials](./tutorials)! 

## Contributing

Interested in helping develop this library, consult our [contributing file](./CONTRIBUTING.md)
for more information!


