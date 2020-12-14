=============
kitcar-rosbag
=============

A ROS_ workspace with scripts to record, replay and modify rosbags.

.. _ROS: https://www.ros.org/

The repository's usage is licensed under a MIT license (''LICENSE'').
If external files are used, a specific LICENSE-file is provided in the same folder,
covering the usage of those files.

The master branch of this repository is mirrored to Github and is publicly available.
The email address and names of all committers will be publicly visible.

.. readme_installation

Installation
============

The kitcar-rosbag scripts have been developed and primarily used on Ubuntu 20.04.
Other Linux distributions were not tested.
Additionally, `ROS Installation <http://wiki.ros.org/ROS/Installation>`_ \
must be installed.


ROS must be installed on your machine.
If it's not yet installed, follow the `installation guide <http://wiki.ros.org/ROS/Installation>`_.


To install required packages run the init script. The packages are installed for the current user.
Change into `kitcar-rosbag` folder and run the script::

   cd kitcar-rosbag
   pip3 install -r requirements.txt

If you do not use the ``bashrc`` provided in ``kitcar-init``, you also have to let ROS know
that this repository exists.
Do this by adding::

   source PATH_TO/kitcar-rosbag/init/bashrc --extend

to your ``~/.bashrc``.

Development
-----------

If you intend on developing this repository, you should also make sure that you install the necessary
pre-commit hooks::

  pip3 install pre-commit && pre-commit install


Build
-----

Then build `kitcar-rosbag` by calling `catkin_make` in the folder of this repository.
