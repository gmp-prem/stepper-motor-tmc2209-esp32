#include <Arduino.h>
#include "ArduinoHardware.h"
#include <HardwareSerial.h>

#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>

#include <TMCStepper.h>
#include "SpeedyStepper.h"

#define DIAG_PIN 18 // STALL motor 2
#define EN_PIN 5    // Enable
#define DIR_PIN 23  // Direction
#define STEP_PIN 22 // Step
#define SPREAD_PIN 19
#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define SERIAL2_RX 16
#define SERIAL2_TX 17
#define DRIVER_ADDRESS 0b00  // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f        // R_SENSE for current calculation
#define STALL_VALUE_HOME 38  // prev:20, For homing, [0..255] if (SG_THRS*2 > SG_RESULT) stall detected
#define STALL_VALUE_NORMAL 0 // For normal operation

#define BAUDRATE 115200
#define BUILTIN_LED_PIN 2 // esp32 led built-in pin

#define RATE 10

#define MICROSTEP_HOMING 2
#define MICROSTEP_NORMAL 0
#define DEFAULT_SP 2500
#define DEFAULT_SP_MM 50
#define DEFAULT_ACC 8000
#define RMS_NORMAL 700 // when motor moving
#define RMS_HOMING 600

using namespace TMC2208_n;

double goal_position, goal_in_step_position;
double speedinmm_current = DEFAULT_SP_MM;
uint8_t gripper_state_value = 0;
uint32_t previous_time = 0;

char buffer[20]; // for check step per mill

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
SpeedyStepper stepper;

// prototype functions
void init_stepper();
void initHomeCallback(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void gripperSpeedCallback(const std_msgs::Float64 &msg);
void gripperSpeedInMMCallback(const std_msgs::Float64 &msg);
void gripperPositionCallback(const std_msgs::Float64 &msg);
void gripperPositionInStepCallback(const std_msgs::Float64 &msg);

// ===== ROS ======
ros::NodeHandle nh;

// msgs
std_msgs::Float64 gripper_position_goal_msg, gripper_position_current_msg, gripper_speedinmm_current_msg;
std_msgs::UInt8 gripper_state_msg;

// subscribers
ros::Subscriber<std_msgs::Float64> gripper_speed_sub("gripper/speed_sub", &gripperSpeedCallback);
ros::Subscriber<std_msgs::Float64> gripper_speedinmm_sub("gripper/speed_inmm_sub", &gripperSpeedInMMCallback);
ros::Subscriber<std_msgs::Float64> gripper_position_sub("gripper/position_in_mm_sub", &gripperPositionCallback);
ros::Subscriber<std_msgs::Float64> gripper_position_in_step_sub("gripper/position_in_step_sub", &gripperPositionInStepCallback);

// publishers
ros::Publisher gripper_position_goal("gripper/position_goal", &gripper_position_goal_msg);
ros::Publisher gripper_position_current("gripper/position_current", &gripper_position_current_msg);
ros::Publisher gripper_speedinmm_current("gripper/speedinmm_current", &gripper_speedinmm_current_msg);
ros::Publisher gripper_state("gripper/state", &gripper_state_msg);

// services
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> serviceServer("gripper/init_home", initHomeCallback);

void gripperSpeedCallback(const std_msgs::Float64 &msg)
{
  digitalWrite(EN_PIN, HIGH);
  delay(1);
  stepper.setSpeedInStepsPerSecond(float(msg.data));
  digitalWrite(EN_PIN, LOW);
}

void gripperSpeedInMMCallback(const std_msgs::Float64 &msg)
{
  speedinmm_current = msg.data;
  digitalWrite(EN_PIN, HIGH);
  delay(1);
  stepper.setSpeedInMillimetersPerSecond(float(speedinmm_current));
  digitalWrite(EN_PIN, LOW);
}

void gripperPositionCallback(const std_msgs::Float64 &msg)
{ 
  goal_position = msg.data;
  nh.loginfo("gripper recieved goal");

  stepper.setupMoveInMillimeters(goal_position);

}

void gripperPositionInStepCallback(const std_msgs::Float64 &msg)
{
  goal_in_step_position = msg.data;
  stepper.setupMoveInSteps(goal_in_step_position);
}

void initHomeCallback(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{ 
  init_stepper();
  nh.loginfo("Homing Finished");
}

// ===== END ROS ======

void driverParamInit()
{
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(800);             // mA max 1.5
  driver.I_scale_analog(true);         // true: Use voltage supplied to VREF as current reference
  driver.microsteps(MICROSTEP_HOMING); // up to 256 or 1/256
  driver.TCOOLTHRS(0xFFFFF);           // 20bit max
  driver.semin(0);                     // 5
  driver.semax(1);
  driver.sedn(0b01);
  driver.pdn_disable(false);   // true: PDN_UART input function disabled. Set this bit, when using the UART interface
  driver.pwm_autoscale(true); // Needed for stealthChop
  // driver.CHOPCONF()
  // driver.shaft(false);            // true=CW/open, false=CCW/close
  // driver.mres(128);
}

void driverCheckConnection()
{
  nh.loginfo("Testing tmc2209-esp32 connection...");
  uint8_t connection_result = driver.test_connection();
  if (connection_result)
  {
    nh.loginfo("connection failed");

    switch (connection_result)
    {
    case 1:
      nh.loginfo("Cause: loose connection");
      break;
    case 2:
      nh.loginfo("Cause: no power");
      break;
    }

    nh.loginfo("Fix the problem and reset board.");
    abort();
  }
  nh.loginfo("tmc2209-esp32 connection is OK");
}

void initHoming()
{
  nh.loginfo("Homing initialized...");

  driver.SGTHRS(STALL_VALUE_HOME);
  driver.rms_current(RMS_HOMING);
  driver.mres(8);
  delay(2);
  digitalWrite(SPREAD_PIN, HIGH); // HIGH=Spread mode, LOW=Stealth mode
  if (driver.stealth())
    nh.loginfo("MODE STEALTH");
  else
    nh.loginfo("MODE SPREADCYCLE");
  stepper.setSpeedInMillimetersPerSecond(DEFAULT_SP_MM);
  stepper.setSpeedInStepsPerSecond(800);
  stepper.setAccelerationInStepsPerSecondPerSecond(400);
  delay(1);
  stepper.setCurrentPositionInSteps(0);
  stepper.setupMoveInSteps(-3000);
  digitalWrite(SPREAD_PIN, HIGH);
  if (driver.stealth())
    nh.loginfo("MODE STEALTH");
  else
    nh.loginfo("MODE SPREADCYCLE");
  stepper.setupMoveInSteps(-3000);
  driver.semin(0); // 5
  driver.semax(1);
  delay(10);
  
  while (!stepper.processMovement())
  {
    if (digitalRead(DIAG_PIN) == 1)
    {
      stepper.setupStop();
      nh.loginfo("Stall Detected !");
      break;
    }
  }

  stepper.setCurrentPositionInSteps(0);
  stepper.setupMoveInSteps(20);

  while (!stepper.motionComplete())
  {
    stepper.processMovement();
  }

  stepper.setCurrentPositionInSteps(0);
  stepper.setCurrentPositionInMillimeters(0);
  stepper.setupMoveInMillimeters(0);
  stepper.setupMoveInSteps(0);
  driver.mres(64);
  driver.semin(0); // 5
  driver.semax(0);
  digitalWrite(SPREAD_PIN, HIGH); // HIGH=Spread mode, LOW=Stealth mode
}

void init_stepper()
{
  pinMode(DIAG_PIN, INPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BUILTIN_LED_PIN, OUTPUT);
  pinMode(SPREAD_PIN, OUTPUT);

  digitalWrite(EN_PIN, HIGH);
  

  // driver
  driver.begin();
  driverParamInit();
  driverCheckConnection();
  delay(5);

  // stepper begin
  stepper.connectToPins(STEP_PIN, DIR_PIN);

  // homing
  stepper.stepsPerMillimeter = 25;
  digitalWrite(EN_PIN, LOW);
  initHoming();
  delay(5);
  digitalWrite(EN_PIN, HIGH);         // after homing, close output and setup again
  nh.loginfo("Homing Finished");
  driver.SGTHRS(STALL_VALUE_NORMAL);  // normal operation
  driver.microsteps(MICROSTEP_NORMAL);
  driver.rms_current(RMS_NORMAL);

  stepper.stepsPerMillimeter = (25 * 80) / 140;

  if (driver.stealth())
    nh.loginfo("MODE STEALTH");
  else
    nh.loginfo("MODE SPREADCYCLE");

  nh.loginfo("Stepper Motor is ready...");
  // Serial.println( "", driver.stealth());

  stepper.setSpeedInStepsPerSecond(DEFAULT_SP);
  stepper.setSpeedInMillimetersPerSecond(DEFAULT_SP_MM);
  stepper.setAccelerationInStepsPerSecondPerSecond(DEFAULT_ACC);
  delay(5);

  // sprintf(buffer, "step per mill: %f", stepper.stepsPerMillimeter); // def: 25
  // Serial.println(buffer);

  digitalWrite(EN_PIN, LOW);
}

void setup()
{
  Serial.begin(BAUDRATE);
  while (!Serial);
  nh.loginfo("Serial connected");

  nh.getHardware()->setBaud(BAUDRATE);
  nh.initNode();
  
  // publishers
  nh.advertise(gripper_position_goal);
  nh.advertise(gripper_position_current);
  nh.advertise(gripper_speedinmm_current);
  nh.advertise(gripper_state);
  // subscribers
  nh.subscribe(gripper_speed_sub);
  nh.subscribe(gripper_speedinmm_sub);
  nh.subscribe(gripper_position_sub);
  nh.subscribe(gripper_position_in_step_sub);
  // service
  nh.advertiseService(serviceServer);

  SERIAL_PORT.begin(BAUDRATE, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX);

  // wait until ros node is actually connected then init the gripper
  while (!nh.connected()) {
    nh.spinOnce();
  } 

  init_stepper();
}

void loop()
{
  uint32_t current_time = millis();

  if ((current_time - previous_time) > RATE)
  { // RATE is 10 ms
    previous_time = current_time;

    gripper_position_goal_msg.data = abs(goal_position - stepper.getCurrentPositionInMillimeters());
    if (gripper_position_goal_msg.data <= 0.1) {
      gripper_state_msg.data = 0;
    }
    else {
      gripper_state_msg.data = 1;
    }

    gripper_speedinmm_current_msg.data = speedinmm_current;

    gripper_position_goal.publish(&gripper_position_goal_msg);
    gripper_position_current.publish(&gripper_position_current_msg);
    gripper_speedinmm_current.publish(&gripper_speedinmm_current_msg);
    gripper_state.publish(&gripper_state_msg);

    nh.spinOnce(); // update callback
  }
  stepper.processMovement();
}