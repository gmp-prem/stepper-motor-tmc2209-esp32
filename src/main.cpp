#include <Arduino.h>
#include <HardwareSerial.h>
#include "ArduinoHardware.h" 

#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <TMCStepper.h>
#include "SpeedyStepper.h"

// declare the variable
#define DIAG_PIN 22
#define EN_PIN 18
#define STEP_PIN 19
#define DIR_PIN 21
#define SERIAL_PORT Serial2
#define BUILTIN_LED_PIN 13
#define RST_PIN 2 // for reset esp32 board

#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f       // R_SENSE for current calculation
#define STALL_HOMING_VALUE 5     // For homing, [0..255] if (SG_THRS*2 > SG_RESULT) stall detected
#define STALL_OPERATION_VALUE 5
#define MICROSTEP_HOMING 2
#define MICROSTEP_NORMAL 0

#define BAUDRATE 115200
#define RATE 500

#define DEFAULT_SP 1000
#define DEFAULT_SP_MM 25
#define DEFAULT_ACC 500

using namespace TMC2208_n;

// global variables
double speed_in_mm_current = DEFAULT_SP_MM;
double position_in_mm_current;
double goal_position;
char buffer[20];

// prototype functions
void init_stepper();

// TMC2209 driver
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS); // for tmc2209 driver
SpeedyStepper stepper; // for stepper motor

// ========== ROS ================
ros::NodeHandle nh;

// msg for publishers
std_msgs::Float64 gripper_goal_position_in_mm_msg;
std_msgs::Float64 gripper_current_speed_in_mm_msg;

void gripperSpeedInMMCallback(const std_msgs::Float64 &msg) {
  speed_in_mm_current = msg.data;
  stepper.setSpeedInMillimetersPerSecond(speed_in_mm_current);
}

void gripperPositionInMMCallback(const std_msgs::Float64 &msg){
  // motorReadyToUseState()
  goal_position = msg.data;

  stepper.setupMoveInMillimeters(goal_position);
}

// publishers
ros::Publisher gripper_goal_position_in_mm("gripper/position_goal", &gripper_goal_position_in_mm_msg);
ros::Publisher gripper_current_speed_in_mm("gripper/current_speed_in_mm", &gripper_current_speed_in_mm_msg);

// subscribers
ros::Subscriber<std_msgs::Float64> gripper_speed_in_mm_sub("gripper/speed_in_mm_sub", &gripperSpeedInMMCallback);
ros::Subscriber<std_msgs::Float64> gripper_position_in_mm_sub("gripper/position_in_mm_sub", &gripperPositionInMMCallback);

// ===============================

void driverParamInit()
{
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(800);    // mA max 1.5
  driver.I_scale_analog(true); // true: Use voltage supplied to VREF as current reference
  driver.TCOOLTHRS(0xFFFFF);   // 20bit max
  driver.semin(5);             // 5
  driver.semax(2);
  driver.sedn(0b01);
  driver.pdn_disable(true);   // true: PDN_UART input function disabled. Set this bit, when using the UART interface
  driver.pwm_autoscale(true); // Needed for stealthChop
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

void initHoming() {
  nh.loginfo("Homing initialized...");

  driver.SGTHRS(STALL_HOMING_VALUE);
  driver.microsteps(MICROSTEP_HOMING);

  stepper.setSpeedInStepsPerSecond(800);
  stepper.setAccelerationInStepsPerSecondPerSecond(400);
  stepper.setCurrentPositionInSteps(0);
  stepper.setupMoveInSteps(-5000);

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
  stepper.setupMoveInSteps(10);

  while (!stepper.motionComplete())
  {
    stepper.processMovement();
  }

  stepper.setCurrentPositionInSteps(0);
  stepper.setCurrentPositionInMillimeters(0);
  stepper.setupMoveInMillimeters(0);
  stepper.setupMoveInSteps(0);

  driver.SGTHRS(STALL_OPERATION_VALUE);
  driver.microsteps(MICROSTEP_NORMAL);
}

void initHomeCallback(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  init_stepper();
  nh.loginfo("Homing Finished");
}

void resetBoardCallback(const std_srvs::Empty::Request&req, std_srvs::Empty::Response &res){
  digitalWrite(RST_PIN, LOW);
  nh.logwarn("REBOOTING ESP32...");
  delay(500);
  digitalWrite(RST_PIN, HIGH);
  nh.logwarn("BOOTED ESP32");

}

ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> serviceServer("gripper/init_home", initHomeCallback);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> serviceServer2("gripper/reset_board", resetBoardCallback);
void init_stepper()
{
  // set pin mode
  pinMode(DIAG_PIN, INPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BUILTIN_LED_PIN, OUTPUT);

  pinMode(RST_PIN, OUTPUT); // low to reset the esp32
  digitalWrite(RST_PIN, HIGH);

  digitalWrite(EN_PIN, HIGH);
  delay(10);
  digitalWrite(EN_PIN, LOW);

  // driver
  driver.begin();
  driverParamInit();
  driverCheckConnection();

  // stepper
  stepper.connectToPins(STEP_PIN, DIR_PIN);
  delay(10);

  // homing (initial stepper value inside homing)
  initHoming();

  // set final speed of motor
  stepper.setSpeedInStepsPerSecond(DEFAULT_SP);
  stepper.setSpeedInMillimetersPerSecond(DEFAULT_SP_MM);
  stepper.setAccelerationInStepsPerSecondPerSecond(DEFAULT_ACC);

  digitalWrite(EN_PIN, LOW); // enable output
}

void setup()
{
  // setup function run once
  Serial.begin(BAUDRATE);
  while (!Serial);

  // Node handle setup 
  nh.getHardware()->setBaud(BAUDRATE);
  nh.initNode();

  nh.advertise(gripper_goal_position_in_mm);
  nh.advertise(gripper_current_speed_in_mm);
  nh.subscribe(gripper_speed_in_mm_sub);
  nh.subscribe(gripper_position_in_mm_sub);
  nh.advertiseService(serviceServer);
  nh.advertiseService(serviceServer2);

  // wait until node is connected
  while(!nh.connected()) {
    nh.spinOnce();
  }
  nh.loginfo("Node started");

  // digitalWrite(RST_PIN, LOW);
  // delay(150);
  // digitalWrite(RST_PIN, HIGH);

  SERIAL_PORT.begin(BAUDRATE);

  init_stepper();


  
}

uint32_t previous_time = 0;
void loop() {
  uint32_t current_time = millis();

  // time event loop, publish something every period of time
  if ((current_time - previous_time) > RATE) { // repeated every 10 ms
    previous_time = current_time;

    gripper_goal_position_in_mm_msg.data = goal_position - stepper.getCurrentPositionInMillimeters();
    gripper_goal_position_in_mm_msg.data = abs(gripper_goal_position_in_mm_msg.data);
    if (gripper_goal_position_in_mm_msg.data == 0.0) {
      delay(1);
      digitalWrite(EN_PIN, HIGH); // disable output (not sure position will change?)
      driver.pdn_disable(LOW); // current reduction enable
    }
    else {
      delay(1);
      digitalWrite(EN_PIN, LOW);
      driver.pdn_disable(HIGH);
    }

    gripper_current_speed_in_mm_msg.data = speed_in_mm_current;

    gripper_goal_position_in_mm.publish(&gripper_goal_position_in_mm_msg);
    gripper_current_speed_in_mm.publish(&gripper_current_speed_in_mm_msg);
  }

  stepper.processMovement(); // makes motor rotate
  nh.spinOnce(); // update ros callbacks
}
