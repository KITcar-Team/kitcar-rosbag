=============
kitcar-rosbag
=============

A ROS_ workspace with scripts to record, replay and modify rosbags.

.. _ROS: https://www.ros.org/

The repositories usage is licensed under a MIT license (''LICENSE'').
If external files are used, a specific LICENSE-file is provided in the same folder, covering the usage of those files.

.. readme_installation

Installation
============

The simulation has been developed and primarily used on Ubuntu 18.04 and Ubuntu 20.04.
Other Linux distributions were not tested.
Additionally, `ROS Installation <http://wiki.ros.org/ROS/Installation>`_ \
must be installed.

Clone
-----

The first step is of course to clone the repository.
These are some ways to get it:

* **KITcar internal**. Clone this repository in the same directory as `kitcar-ros`.
  Change into the correct directory. By default it's ``/home/<USERNAME>/kitcar`` and run::

   git clone git@git.kitcar-team.de:kitcar/kitcar-rosbag.git


$KITCAR_REPO_PATH
-----------------

The environment variable **$KITCAR_REPO_PATH** must contain the directory in which you've cloned **kitcar-rosbag**.

Make sure that **$KITCAR_REPO_PATH** is set to the directory where you've cloned **kitcar-rosbag** into::

  cd $KITCAR_REPO_PATH/kitcar-rosbag

should put you into the root directory of **kitcar-rosbag**.

If it doesn't work, create the variable with::


   export KITCAR_REPO_PATH=<DIRECTORY WHERE kitcar-rosbag IS>


Adding

::

  export KITCAR_REPO_PATH=<DIRECTORY WHERE kitcar-rosbag IS>

to your ``.bashrc`` ensures that the path is always set.


ROS
---

ROS must also be installed on your machine.
If it's not yet installed, follow the `installation guide <http://wiki.ros.org/ROS/Installation>`_.

Init-Script
-----------

To install required packages run the init script. The packages are installed for the current user.
Change into `kitcar-rosbag` folder and run the script::

   cd $KITCAR_REPO_PATH/kitcar-rosbag
   ./init/init.sh

(*Ubuntu 18.04: Ignore any error thrown by pip when trying to install pygobject, it seems to be irrelevant.*)

If you want to, you can also install packages that are used to run machine learning tasks or compile the documentation.
But these packages are not needed to run all basic components of the simulation.

Build
-----

Then build `kitcar-rosbag` by calling `catkin_make` in the folder of this repository.
