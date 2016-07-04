# tankbot
Arduino based robot with basic navigation abilities

Tankbot started out as one of my first Arduino-based projects and it is based on a set of exercises in "Arduino Workshop" by John Boxall. This is a great book if you are starting out with Arduino programming. Tankbot is still a work in progress and is designed to use an Arduino Mega (ATMega2560) so that there is plenty of room for expansion.

After getting Tankbot to work with ultrasonic and infrared distance sensors I added an accelerometer to detect tilting and bump sensors on the from of the robot attached to an interrupt pin. Tankbot's ultrasonic sensor is mounted on a pan/tilt kit to enable it to "scan" the surrounding environment. Tankbot creates an array of distance values for each position of the pan servo (so 180 positions) and then uses these values and some basic maths to find "gaps" that it can fit through. The infrared distance sensor is mounted on the from of the robot and used to detect whether Tankbot is getting too close to an object in front. The bump sensors take the form of whiskers that protect the front edges of the robot. The accelerometer is used to tell if Tankbot is tilting significantly to one side.

If Tankbot detects an obstacle it will reverse and then turn to a random position (this prevents it from repeating the same movements over and over - for example when stuck in a corner).
