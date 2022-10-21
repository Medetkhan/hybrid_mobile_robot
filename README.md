# hybrid_mobile_robot
Peripherals for the robot based on tensegrity.

Robot has rear wheel and additional passive wheels to help tensegrity legs crawl.
Legs are made as tensegrity to absorb impact and thus prevent from serious damage the ost fragile part - electronics and equipment.
Absorption would tested by cascade of IMU sensors on one of the legs connected to ESP32 microcontroller via multiplexer.
Raspberry PI4 would be used to run python script for UDP video streaming via Hemisphere camera (fish eye). And also python Dynamixels SDK will be used to control motors to make legs move.
It can be used in various applications such as find and resque missons and etc.  
