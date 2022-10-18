# hybrid_mobile_robot
Peripherals for the robot based on tensegrity.


Fish_Eye camera (Hemosphere camera) that would be our eyes of the robot. This USB camera is connected to raspberry Pi4 with Rasbian OS.
Video Streaming was implemented that streams video via TCP/IP. Previously 30 fps has been reached but now some bugs occurs.
We tried also RTSP (Real time Streaming protocol) with VLC media player on the PC,however there is always 1.5 sec latency even with local hotspot.
Ad-Hoc Mode has ben implemented but with no big result.
