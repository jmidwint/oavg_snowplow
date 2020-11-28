/*
 * OAVG ROS motor control
 * - Receives ROS Twist message on "oavg_motor_cmd/cmd_vel" topic
 * 
 * 
*/

// Define motor driver pins
// A = Power, B = Direction
#define RIGHT_A 4  // Right motor power
#define RIGHT_B 2  // Right direction
#define LEFT_A  5  // Left motor power
#define LEFT_B  0  // Left direction

#define MAX_MOTOR_POWER 1023

#define USE_SERIAL Serial

//#define USE_ROS false
//#define USE_ROS true


// To use the TCP version of rosserial_arduino
#define ROSSERIAL_ARDUINO_TCP

#include <Arduino.h>

#include <ros2.h>
#include <geometry_msgs/Twist.h>

//#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;
std_msgs::String str_msg;
geometry_msgs::Twist twist_msg;


//const bool use_ros=USE_ROS;

// ------------------------------------
// Set your WiFi SSID and Password here
// ------------------------------------

const char* ssid = "MakerSpaceNorth";
const char* password = "EnableMakers";

// Set wifiAP to TRUE to have ESP act as Access Point
// If set to FALSE, then connect to SSID defined above
const bool  wifiAP = false;

// Define Ros master IP address
IPAddress rosServer(192,168,86,241);      // Nano (home) - Set the rosserial socket ROSCORE SERVER IP address

const uint16_t rosServerPort = 11411;    // Set the rosserial socket server port

uint8_t driveFwdDirection = HIGH;

/*
 * Motors Motion control for differential steering vehicles
 * 
 * Xaxis to steering, Yaxis Throttle
 * value from -100 to 100 
 */
void motionControl(int Xaxis, int Yaxis){

  int motorRight = 0, motorRightPower = 0;
  int motorLeft = 0, motorLeftPower = 0;

  //Set motors directions based on Yaxis 

  if (Yaxis > 0){
    motorLeft = Yaxis+Xaxis;
    motorRight = Yaxis-Xaxis;
  } else {
    motorLeft = Yaxis-Xaxis;
    motorRight = Yaxis+Xaxis;
  }

  digitalWrite(LEFT_B, (motorLeft > 0) ? driveFwdDirection : !driveFwdDirection);
  digitalWrite(RIGHT_B, (motorRight > 0) ? driveFwdDirection : !driveFwdDirection);

  motorLeft = abs(motorLeft);
  motorRight = abs (motorRight);

  if (motorLeft > 100) motorLeft=100;
  if (motorRight > 100) motorRight=100;

  // Map motor values 0-100% to 0-1023 (Max motor power)
  motorLeftPower = map(motorLeft, 0, 100, 0, MAX_MOTOR_POWER);
  motorRightPower = map(motorRight, 0, 100, 0, MAX_MOTOR_POWER);
  
  Serial.print("MotorAxis: X=");
  Serial.print(  Xaxis);
  Serial.print(", Y=");
  Serial.println( Yaxis);

  Serial.print("MotorPower: R=");
  Serial.print(  motorRightPower);
  Serial.print(", L=");
  Serial.println( motorLeftPower);

  // Set Motor Duties based on calculation
  analogWrite(LEFT_A, motorLeftPower);
  analogWrite(RIGHT_A, motorRightPower);
}


// Callback for geometry twist message
// Parse linear and angular velocities and send to motion control 

void on_cmd_vel( const geometry_msgs::Twist& msg){

  float X, Y;

  // Note: Joystick limits set as 0 - 2.  Multiply by 50 to get range 0 - 100%
  X = msg.angular.z*50;
  Y = msg.linear.x*50;
  
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
  Serial.println(millis());
  Serial.print("Received cmd_vel: X=");
  Serial.print(  msg.linear.x*50);
  Serial.print(", Y=");
  Serial.println( msg.angular.z*50);

  motionControl((int) X, (int) Y) ;
}

ros::Subscriber<geometry_msgs::Twist> sub("oavg_motor_cmd/cmd_vel", &on_cmd_vel );


void setup() {
  USE_SERIAL.begin(115200);

  //USE_SERIAL.setDebugOutput(true);

  if (wifiAP) {
  
   USE_SERIAL.print("Setting soft-AP ... ");
    boolean result = WiFi.softAP("wificar");
    if(result == true)
    {
      USE_SERIAL.println("Ready");
      USE_SERIAL.printf("Soft-AP IP address = \n");
      USE_SERIAL.println(WiFi.softAPIP());
    }
    else
    {
      USE_SERIAL.println("Failed!");
    }
  }
  else
  {
    WiFi.hostname("oavg_v01");
    WiFi.begin(ssid, password);
    Serial.println("");
  
   
    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      USE_SERIAL.print(".");
    }
    USE_SERIAL.println("");
    USE_SERIAL.print("Connected to ");
    USE_SERIAL.println(ssid);
    USE_SERIAL.print("IP address: ");
    USE_SERIAL.println(WiFi.localIP());
  }
  
  for (uint8_t t = 4; t > 0; t--) {
    USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
    USE_SERIAL.flush();
    delay(1000);
  }

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(rosServer, rosServerPort);
  nh.initNode();

  nh.subscribe(sub);

  // Display local and Ros-master IP addresses
  Serial.print("Local IP = ");
  Serial.println(nh.getHardware()->getLocalIP());
  Serial.print("Ros-master IP = ");
  Serial.println(rosServer);

  // Set motor control pins as Outputs
  pinMode(LEFT_A, OUTPUT);
  pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);

  // Init motors
  motionControl(0, 0);

}

void loop() {

  nh.spinOnce();
  delay(1);

}
