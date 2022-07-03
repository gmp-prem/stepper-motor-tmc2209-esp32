## ESP32 + TMC2209 + StepperMotor(NEMA17)
Using a TMC2209 stepper driver module with stepper motor NEMA17 on the ESP32 for homing position

## how to run the gripper

1. turn on the power supply for tmc2209

2. open your favorite terminal
```
ctrl+alt+t
```
3. run the rosserial manully
```
rosrun rosserial_arduino serial_node.py _port:=/dev/ttyUSB_STEPPERGRIPPER _baud:=115200
```