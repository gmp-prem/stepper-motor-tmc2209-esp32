#include <Arduino.h>
#include "ArduinoHardware.h"
#include <HardwareSerial.h>

#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <TMCStepper.h>
#include "SpeedyStepper.h"

#define DIAG_PIN         18           // STALL motor 2
#define EN_PIN           5            // Enable
#define DIR_PIN          23           // Direction
#define STEP_PIN         22           // Step
#define SERIAL_PORT      Serial2      // TMC2208/TMC2224 HardwareSerial port
#define SERIAL2_RX       16
#define SERIAL2_TX       17
#define DRIVER_ADDRESS   0b00         // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE          0.11f        // R_SENSE for current calculation
#define STALL_VALUE      20            // [0..255] if (SG_THRS*2 > SG_RESULT) stall detected
#define SPREAD_PIN 19

#define BAUDRATE 115200
#define BUILTIN_LED_PIN 2 // esp32 led built-in pin

#define RATE 100

using namespace TMC2208_n;

double goal_position, goal_in_step_position;

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE , DRIVER_ADDRESS);
SpeedyStepper stepper;

// ===== ROS ======  

ros::NodeHandle nh;

// TODO: also make speed in
void gripperSpeedCallback (const std_msgs::Float64& msg) {
  stepper.setSpeedInStepsPerSecond(msg.data);
}

// TODO: control how wide in cm
void gripperPositionCallback (const std_msgs::Float64& msg) {
  goal_position = msg.data;
  stepper.setupMoveInMillimeters(goal_position);
}

void gripperPositionInStepCallback (const std_msgs::Float64& msg) {
  goal_in_step_position = msg.data;
  stepper.setupMoveInSteps(goal_in_step_position);
}



std_msgs::Float64 gripper_position_goal_msg;
std_msgs::Float64 gripper_position_current_msg;
ros::Subscriber<std_msgs::Float64> gripper_speed_sub("gripper/speed_sub", &gripperSpeedCallback);
ros::Subscriber<std_msgs::Float64> gripper_position_sub("gripper/position_in_mm_sub", &gripperPositionCallback);
ros::Subscriber<std_msgs::Float64> gripper_position_in_step_sub("gripper/position_in_step_sub", &gripperPositionInStepCallback);
ros::Publisher gripper_position_goal("gripper/position_goal", &gripper_position_goal_msg);
ros::Publisher gripper_position_current("gripper/position_current", &gripper_position_current_msg);



// =============

void driverParamInit() {
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(1400);        // mA max 1.5
  driver.I_scale_analog(true);    // true: Use voltage supplied to VREF as current reference
  driver.microsteps(2);           // up to 256 or 1/256
  driver.TCOOLTHRS(0xFFFFF);      // 20bit max
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.pdn_disable(true);       // true: PDN_UART input function disabled. Set this bit, when using the UART interface
  driver.pwm_autoscale(true);     // Needed for stealthChop
  // driver.CHOPCONF()
  // driver.shaft(false);            // true=CW/open, false=CCW/close
  
  driver.mres(128);
}

void driverCheckConnection() {
  nh.loginfo("Testing tmc2209-esp32 connection...");
  uint8_t connection_result = driver.test_connection();
  if (connection_result) {
      nh.loginfo("connection failed");

      switch(connection_result) {
          case 1: nh.loginfo("Cause: loose connection"); break;
          case 2: nh.loginfo("Cause: no power"); break;
      }
 
      nh.loginfo("Fix the problem and reset board.");
      abort();
  }
  nh.loginfo("tmc2209-esp32 connection is OK");
}

void initHoming () {
  nh.loginfo("Homing initialized...");
  driver.SGTHRS(20);
  
  stepper.setSpeedInStepsPerSecond(800);
  stepper.setAccelerationInStepsPerSecondPerSecond(400);
  stepper.setCurrentPositionInSteps(0);
  stepper.setupMoveInSteps(-5000);

  while (!stepper.processMovement()) {
    if (digitalRead(DIAG_PIN) == 1) {
      stepper.setupStop();
      nh.loginfo("Stall Detected !");
      break;
    }
  }

  stepper.setCurrentPositionInSteps(0);
  stepper.setupMoveInSteps(5);

  while (!stepper.motionComplete()) {
    stepper.processMovement();
  }
  
  stepper.setCurrentPositionInSteps(0);
  stepper.setCurrentPositionInMillimeters(0);
  stepper.setupMoveInMillimeters(0);
  stepper.setupMoveInSteps(0);
}

void initHomeCallback(const std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {

  initHoming();
  nh.loginfo("Homing Finished");
  driver.SGTHRS(0); // normal operation
  driver.microsteps(0);
  nh.loginfo("Stepper Motor is ready...");
}

ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> serviceServer("gripper/init_home", initHomeCallback);

void setup() {
  Serial.begin(BAUDRATE);
  while(!Serial);
  nh.loginfo("Serial connected");

  nh.getHardware()->setBaud(BAUDRATE);
  nh.initNode();
  nh.advertise(gripper_position_goal);
  nh.advertise(gripper_position_current);
  nh.subscribe(gripper_speed_sub);
  nh.subscribe(gripper_position_sub);
  nh.subscribe(gripper_position_in_step_sub);
  nh.advertiseService(serviceServer);

  SERIAL_PORT.begin(BAUDRATE, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX);

  pinMode(DIAG_PIN ,INPUT);
  pinMode(EN_PIN ,OUTPUT);
  pinMode(STEP_PIN ,OUTPUT);
  pinMode(DIR_PIN ,OUTPUT);
  pinMode(BUILTIN_LED_PIN, OUTPUT);
  pinMode(SPREAD_PIN, OUTPUT);
  digitalWrite(SPREAD_PIN, HIGH);
  digitalWrite(EN_PIN, HIGH);
  delay(500);
  digitalWrite(EN_PIN, LOW);

  // driver
  driver.begin();
  driverParamInit();
  driverCheckConnection();
  
  // stepper begin
  stepper.connectToPins(STEP_PIN, DIR_PIN);

  // homing
  initHoming();
  nh.loginfo("Homing Finished");
  driver.SGTHRS(0); // normal operation
  driver.microsteps(0);
  nh.loginfo("Stepper Motor is ready...");
  
  if (driver.stealth())
    nh.loginfo("STEALTH!");
  else
    nh.loginfo("NON STEALTH!");

  // Serial.println( "", driver.stealth());
}

uint32_t previous_time = 0;

void loop () {
  uint32_t current_time = millis();
  // stepper.setAccelerationInStepsPerSecondPerSecond(300); 
  stepper.setAccelerationInStepsPerSecondPerSecond(8000);

  if((current_time - previous_time) > RATE) { // now RATE is 100 ms
    previous_time = current_time;

    gripper_position_goal_msg.data = goal_position - stepper.getCurrentPositionInMillimeters();
    gripper_position_goal_msg.data = abs(gripper_position_goal_msg.data);

    gripper_position_goal.publish(&gripper_position_goal_msg);
    gripper_position_current.publish(&gripper_position_current_msg);

    nh.spinOnce(); // update callback 
  }
  
  stepper.processMovement();
}