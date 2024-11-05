The github repository for Calvin Dai, otherwise known as 1averagepythonenjoyer.

Currently working on a PID line follower robot, using an Arduino Nano ESP32 to take in sensor values and execute the PID algorithm, and a Raspberry Pi to do motor control.

It would have been easier just to make the Arduino do all the work, but I had an RPi-based robot from a few months ago used in the PiWars Robotics Competition (at which I led my team to 1st place) with all the hardware in place,
and I didn't want to get rid of it either, so I thought serial communication via USB is probably the easiest way to get round this, instead of completely redoing the hardware. 
