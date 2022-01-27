## ESP32 + TMC2209 + StepperMotor(NEMA17)
Using a TMC2209 stepper driver module with stepper motor NEMA17 on the ESP32 for homing position

_**This repo is not yet completed, further patch might be updated**_

## Datasheet

Technical document: https://wiki.fysetc.com/Silent2209/

Datasheet: https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf

## Starting up

1. Open the first terminal, run roscore
```
roscore
```

2. Anoter terminal, run rosserial node from the current port, **take note that port can be dynamically changed everytime if no port mapping **
```
rosrun rosserial_arduino serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
```

in case that the port is not recognized, check the port connection or change the mode using this command
```
sudo chmod a+rw /dev/ttyUSB*
```
or
```
sudo chmod a+rw /dev/ttyACM*
```

Currently, this rosserial node will subscribe position and speed

**speed topic**:    /gripper/speed_in_mm_sub

**position topic**: /gripper/position_in_mm_sub

3. To control, pubish the message to the following topic

3.1 using ROS GUI
```
rqt
```

3.2 using rostopic pub (your topic)
```
rostopic pub /gripper/position_in_mm_sub std_msgs/Float64 "data: 20.0"

```


