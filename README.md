# Offgrid Arduino

It pains me to release this project into the wild in its current state. In doing so, I hope that others can learn by example, contribute improvements, and create documentation.

This project has been developed using a Raspberry Pi 3B+, "Raspberry Pi Meet Arduino" hat, and a few external components. Reading through the code should indicate these components, but I can try to answer questions on what has been used where it isn't clear. I will not be able to answer troubleshooting questions regarding how to get hardware components connected and operational.


Code for the Raspberry Pi 3B+ board may be found here: https://github.com/TheWandrr/offgrid

Code for programming the Arduino's EEPROM may be found here: https://github.com/TheWandrr/offgrid_init_eeprom

# Building and Uploading

Compile using arduino-cli (package must be installed before using these scripts)

    > ./make.sh

Upload to board

    > ./upload.sh

  *** Note that there may be additional packages to install. Let me know and I'll add them here to help others in the future.

# Purpose

The Arduino is connected via serial link to the Raspberry Pi. The functions of the Arduino are as follows:
    - Manage encoder input and PWM output for light brightness control
    - Constantly monitor energy input/output, calculating and maintaining state of charge
    - Using temperature sensor data and user setpoint, control furnace using simple heating control loop
    - Broadcast metrics/telemetry back to Raspberry Pi for publishing to network and recording to database
    - Accept commands from Raspberry Pi to set various parameters
# Future Direction

Most of the code in this project has been running in my own campervan conversion for at least a year or two. Small changes have been made along the way when I've added new hardware (eg. solar) or have become annoyed with the way something was working (or not). It has arrived at a point where I now see better ways to design the system and thus will probably only use portions of the code herein but with a different system topology.

TheWandrr
2022-05-15
