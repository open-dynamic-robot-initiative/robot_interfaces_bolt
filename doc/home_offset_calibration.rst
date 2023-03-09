***********************
Home Offset Calibration
***********************

Before using the robot, the home offsets need to be calibrated.  This can be
done in the following way:

1. Manually move the joints to be roughly at the desired zero position.
2. Run the following command:

   .. code-block:: bash

      ros2 run solo bolthumanoid_hardware_calibration <network-interface>

   This will find the next encoder indices (this is the 'home position') and
   then start printing the offset.
3. Manually move the joints back to the desired zero position.  This time it has
   to be precise.
4. Stop ``bolthumanoid_hardware_calibration`` (via Ctrl+C).  Copy the last offsets that
   were printed to the terminal and insert them in the configuration file as
   value for ``home_offset_rad`` (see :doc:`config`).
