=============
kitcar-rosbag
=============

A ROS_ workspace with scripts to record, replay and modify rosbags.

.. _ROS: https://www.ros.org/

The repository's usage is licensed under a MIT license (''LICENSE'').
If external files are used, a specific LICENSE-file is provided in the same folder,
covering the usage of those files.

.. readme_installation

Installation
============

The kitcar-rosbag scripts have been developed and primarily used on Ubuntu 20.04.
Other Linux distributions were not tested.
Additionally, `ROS Installation <http://wiki.ros.org/ROS/Installation>`_ \
must be installed.

Prerequisites
---------------

ROS must also be installed on your machine.
If it's not yet installed, follow the `installation guide <http://wiki.ros.org/ROS/Installation>`_.

The environment variable **$KITCAR_REPO_PATH** must be set to the directory
where all your KITcar repositories are located.

Make sure that **$KITCAR_REPO_PATH** is set::

  cd $KITCAR_REPO_PATH

If it doesn't work, create the variable with::


   export KITCAR_REPO_PATH=<DIRECTORY WHERE KITCAR REPOS ARE>


Adding the command to your ``~/.bashrc`` ensures that it will always be set.


Clone
-----

The first step is of course to clone the repository.
These are some ways to get it:

* **KITcar internal**. Clone this repository into your ``$KITCAR_REPO_PATH``::

   git clone git@git.kitcar-team.de:kitcar/kitcar-rosbag.git $KITCAR_REPO_PATH/kitcar-rosbag


Init-Script
-----------

To install required packages run the init script. The packages are installed for the current user.
Change into `kitcar-rosbag` folder and run the script::

   cd $KITCAR_REPO_PATH/kitcar-rosbag
   ./init/init.sh

If you do not use the ``bashrc`` provided in ``kitcar-init``, you also have to let ROS know
that this repository exists.
Do this by adding::

   source $KITCAR_REPO_PATH/kitcar-rosbag/init/bashrc --extend

to your ``~/.bashrc``.


Build
-----

Then build `kitcar-rosbag` by calling `catkin_make` in the folder of this repository.
