arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328
arduino-cli upload --port /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328
cat /dev/ttyUSB0 | less
