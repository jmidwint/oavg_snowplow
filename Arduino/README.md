# ESP8266 ROS Motor control

* This arduino sketch uses the rosserial_arduino library.  Info can be found here: <http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup>

Here's a summary from that website:

>Using the rosserial_arduino package, you can use ROS directly with the Arduino IDE. rosserial provides a ROS communication protocol that works over your Arduino's UART. It allows your Arduino to be a full fledged ROS node which can directly publish and subscribe to ROS messages, publish TF transforms, and get the ROS system time.


# Installation

* Follow the instructions at the link above to install the rosserial ROS package and Arduino SDK
* Generate the rosserial library and copy it to the <sketches>/libraries folder
* Copy the updated "ros2.h" header file to the ros_lib libraries folder.  This updated header file will allow the ESP8266 and ESP32 to use the UART rather than forcing it to TCP.  See issues section below.
* Open the "ESP_ROS_Motor_Control_v01.ino" sketch file in the Arduino SDK.
* Edit the file and change the SSID and WiFi password for your local WiFi network.  Update the IP address of the host where the ros core/master is running.
* Make sure the ESP8266 pins to the motor drivers are correct. By default they are set as: A = Power/PWM, B = Direction
* Upload the sketch to the ESP8266.  The sketch will automatically connect to WiFi network, start a ROS node, and subscribe to the "oavg_motor_cmd/cmd_vel" topic.
* You can check this by going to another ROS node and typing "rostopic list".  You should see the "oavg_motor_cmd/cmd_vel" topic listed.
* You should now be able to send "geometry_msgs::Twist" messages to the "oavg_motor_cmd/cmd_vel" topic and see your motors move!


# ros_lib Issues

* There is an issue with the ros.h header file where ESP8266 and ESP32 devices will automatically be set to use ""ArduinoTcpHardware.h" regardless of the "ROSSERIAL_ARDUINO_TCP" setting.  Normally defining "ROSSERIAL_ARDUINO_TCP" will set rosserial to use TCP, and not defining it will set rosserial to use the UART.  However, for the default ros.h header file the ESP8266 and ESP32 devices are always forced to use TCP.  The changes in the ros2.h header file will revert back to the normal behaviour.  If "ROSSERIAL_ARDUINO_TCP" is not defined, then UART is used even for the ESP8266 and ESP32.


