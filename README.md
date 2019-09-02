# ZERO 2019 Autonomous Vehicle Project

## Installation
* ./install.sh

  

## Run the code
* roscore
  `roscore`

* gps

  1. inject the arduino usb to the usb port

  2. check the port of the usb by typing  `$ ls /dev/ttyACM*`

     If for example, `/dev/ttyACM0` appears we have port of `/dev/ttyACM0`

  3. Start Arduino IDE by typing `$ arduino &`

  4. Find and open Arduino file `gps_and_ros.ino` located in `~/catkin_wd/src/zer2019/main_stream/perception/gps/scripts/gps_and_ros.ino`

  5. In the upper part, click 'Tools' and select port to `/dev/ttyACM0` (in the example)
  
  6. Click verify and click upload button
  
  7. If the IDE outputs `Sketch uses * bytes..` with no red error message, you are complete!
  
  8. Run ros node by typing `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600` (change port if your's is different)
  
  9. Check gps message by `rostopic echo location_msg`
  
     
  
* 

