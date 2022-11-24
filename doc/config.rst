*************
Configuration
*************

You need to create a small configuration file for the robot. The file needs to
be in YAML format and can contain the following values:

-  ``network_interface``: Name of the network interface to which the robot
   is connected (e.g. “eth0”). This parameter is required.

-  ``slider_serial_port``: Name of the serial port to which the hardware
   slider is connected. This can typically be omitted, in which case the
   port is auto-detected.

-  ``max_motor_current_A`` (default: 8 A): Maximum current that can be
   applied to the motors (in Ampere).

-  ``home_offset_rad`` (default: [0, 0, …]): Offset between home position
   (=encoder index) and zero position. See :doc:`home_offset_calibration`.

-  ``logger_level`` (default: “warning”): Controls the verbosity of the
   output. Valid values are {“trace”, “debug”, “info”, “warning”,
   “error”, “critical”, “off”}.


Example:

.. code-block:: yaml

   network_interface: enp0s31f6
   home_offset_rad: [-0.475, -0.589, -0.500, -0.219,  1.496, -0.308,  0.095, -0.319, -0.177, -0.981, -0.924, -1.175]
   logger_level: debug
