Installation
============

Real Time Setup
---------------

For being able to reliably control the robot, a real-time capable kernel
is needed. See :doc:`robot_interfaces:doc/real_time` in the robot_interfaces
documentation.


.. _installation-1:

Installation
------------

We are providing `Apptainer <https://apptainer.org>`__ images for using
the software, so you don’t need to install any dependencies (apart from
Apptainer, of course).

There are two different scenarios described in the following:

-  **Basic Usage:** If you just want to run your own code using this
   package but not modify anything in the robot interfaces packages
   themselves (normal user scenario).
-  **For Development:** If you want to make modifications to the robot
   interfaces packages.

Basic Usage
~~~~~~~~~~~

We provide an Apptainer container with ``robot_interfaces_bolt`` and all
its dependencies installed:

.. code:: bash

   $ apptainer pull oras://ghcr.io/open-dynamic-robot-initiative/trifinger_singularity/solo_robot:latest

.. note::
   The packages in this container are built for real-time
   communication with the robot. This assumes the computer is set up accordingly
   (see `Real Time Setup`_).
   There is an alternative container "solo_user" with a "normal" built, but
   you will likely run into timing-related issues if using it for
   controlling the robot.

You can run test/demo applications directly from that container:

.. code:: bash

   $ apptainer run -e solo_robot.sif ros2 run robot_interfaces_bolt bolthumanoid_show_data ./config.yml

Likewise, you can run custom scripts:

.. code:: bash

   $ apptainer run -e solo_robot.sif python3 ./my_script.py

You can also built custom packages which depend on ``robot_interfaces_bolt``:

.. code:: bash

   # expected directory structure:  ~/workspace/src/my_package
   $ cd ~/workspace
   $ apptainer shell -e path/to/solo_robot.sif
   Apptainer> source /setup.bash  # Needed to setup the environment
   Apptainer> colcon build

   Apptainer> source install/setup.bash  # Needed to be able to use the built package
   Apptainer> # now you can run executables from your package

For Development
~~~~~~~~~~~~~~~

Get the **trifinger_base** Apptainer container (it also contains everything
needed for this package):

.. code:: bash

   $ apptainer pull oras://ghcr.io/open-dynamic-robot-initiative/trifinger_singularity/trifinger_base:latest

Install treep:

.. code:: bash

   $ pip install --upgrade treep

Create a workspace directory, clone the treep configuration and the project:

.. code:: bash

   $ mkdir ~/my_workspace
   $ cd ~/my_workspace

   $ git clone git@github.com:machines-in-motion/treep_machines_in_motion.git 
   $ treep --clone ROBOT_INTERFACES_SOLO

Build the workspace using the Apptainer container:

.. code:: bash

   $ cd ~/my_workspace/workspace
   $ apptainer shell -e path/to/trifinger_user.sif
   Apptainer> source /setup.bash  # Needed to setup the environment
   Apptainer> colcon build

   Apptainer> source install/setup.bash  # Needed to be able to use the built packages

To run an application with Apptainer:

.. code:: bash

   $ cd ~/my_workspace/workspace
   $ apptainer shell -e path/to/trifinger_user.sif
   Apptainer> source install/setup.bash
   # now you can run applications (e.g. one of the demos):
   Apptainer> ros2 run robot_interfaces_bolt demo_bolthumanoid_sine path/to/config.yml

**Note:** When running commands in the following, it is always assumed that this
is done in inside the container with the setup.bash of the workspace sourced.
