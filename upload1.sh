arduino-cli upload -b arduino:avr:leonardo -p /dev/ttyACM1
sleep 2.0
sudo systemctl restart offgrid-daemon.service
