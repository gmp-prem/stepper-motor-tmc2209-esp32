#include <HardwareSerial.h>
#include <TMCStepper.h>


#define DIAG_PIN_2         18           // STALL motor 2
#define EN_PIN_2           5            // Enable
#define DIR_PIN_2          23           // Direction
#define STEP_PIN_2         22           // Step
#define SERIAL_PORT_2      Serial2      // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS_2   0b00         // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE_2          0.11f        // R_SENSE for current calc.  
// Higher STALL_VALUE_2 value -> higher torque to indicate the stall and vice versa
// Lower  STALL_VALUE_2 value -> lower torque to indicate stall detection
// the diag_pin will be 1 when SG_THRS*2 > SG_RESULT
#define STALL_VALUE_2      30       // [0..255] optimized

bool shaft = true;

hw_timer_t * timer1 = NULL;
TMC2209Stepper driver2(&SERIAL_PORT_2, R_SENSE_2 , DRIVER_ADDRESS_2);
using namespace TMC2208_n;

const int ledPin = 2;

void IRAM_ATTR onTimer() {
  digitalWrite(STEP_PIN_2, !digitalRead(STEP_PIN_2));
}

void activate_interrupt(){
  {
    cli();//stop interrupts
    timer1 = timerBegin(3, 8,true); // Initialize timer 4. Se configura el timer,  ESP(0,1,2,3)
                                 // prescaler of 8, y true es una bandera que indica si la interrupcion se realiza en borde o en nivel
    timerAttachInterrupt(timer1, &onTimer, true); //link interrupt with function onTimer
    timerAlarmWrite(timer1, 8000, true); //En esta funcion se define el valor del contador en el cual se genera la interrupción del timer
    timerAlarmEnable(timer1);    //Enable timer        
    sei();//allow interrupts
  }
}

void setup() {
  Serial.begin(115200);         // Init serial port and set baudrate
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");
  SERIAL_PORT_2.begin(115200, SERIAL_8N1, 16, 17); // HW serial btw esp32-tmc2099; baud, config, rx, tx

  pinMode(DIAG_PIN_2 ,INPUT);
  pinMode(EN_PIN_2 ,OUTPUT);
  pinMode(STEP_PIN_2 ,OUTPUT);
  pinMode(DIR_PIN_2 ,OUTPUT); // TODO which dir is rotating respect to motoman part?

  pinMode(ledPin, OUTPUT);

  digitalWrite(EN_PIN_2 ,LOW);
  // digitalWrite(DIR_PIN_2 ,HIGH);

  driver2.begin();
  driver2.toff(4);
  driver2.blank_time(24);
  driver2.rms_current(500); // mA max 1.5
  driver2.I_scale_analog(true); // true: Use voltage supplied to VREF as current reference
  driver2.microsteps(2); // up to 256 or 1/256
  driver2.TCOOLTHRS(0xFFFFF); // 20bit max
  driver2.semin(5);
  driver2.semax(2);
  driver2.sedn(0b01);
  driver2.SGTHRS(STALL_VALUE_2);
  driver2.shaft(false); // true=CW/open, false=CCW/close
  driver2.pdn_disable(true); // true: PDN_UART input function disabled. Set this bit, when using the UART interface
  driver2.pwm_autoscale(true);   // Needed for stealthChop



  delay(100);

  // check connection of esp32-tmc2209
  Serial.println("\nTesting tmc2209-esp32 connection...");
  uint8_t connection_result = driver2.test_connection();
  if (connection_result) {
      Serial.println("connection failed");
      Serial.print("Likely cause: ");

      switch(connection_result) {
          case 1: Serial.println("loose connection"); break;
          case 2: Serial.println("no power"); break;
      }

      Serial.println("Fix the problem and reset board.");
      
      delay(100); // We need this delay or messages above don't get fully printed out
      abort();
  }
  Serial.println("OK"); // if connection is okay

  // activate_interrupt(); // motor always run
}

bool flag = true;

void loop() {
  
  static uint32_t last_time=0;
  uint32_t ms = millis();
  bool diag_bool = digitalRead(DIAG_PIN_2);

  if((ms-last_time) > 100) { //run every 0.1s
    last_time = ms;

    // control stepper via UART
    if (flag) {
      digitalWrite(ledPin, 0); // led off if no stall is detected
      driver2.VACTUAL(500); // set speed of motor
      Serial.print("sg_result: ");
      Serial.print(driver2.SG_RESULT(), DEC); // stallguard value
      Serial.print(" diag: ");
      Serial.println(diag_bool); // diag
    }

    // if ((driver2.SG_RESULT()) < (STALL_VALUE_2*2)) {
    if (diag_bool) {
      digitalWrite(ledPin, 1); // led on if the stall is detected
      driver2.VACTUAL(0);
      Serial.println("Stall detected !");
      flag = false;
      shaft = !shaft;
      driver2.shaft(shaft);
      delay(2000); // delay 2 secs before set flag to true to run again in reverse direction
      flag = true;
    }  
  }

}

