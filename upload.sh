#! /bin/bash

if [ -c /dev/ttyACM0 ]
then
  arduino-cli upload -b arduino:avr:leonardo -p /dev/ttyACM0
elif [ -c /dev/ttyACM1 ]
then
  arduino-cli upload -b arduino:avr:leonardo -p /dev/ttyACM1
else
  echo No devices were found
  exit -1
fi

sleep 2.0
sudo systemctl restart offgrid-daemon.service
