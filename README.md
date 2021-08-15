# ArduPilot Project

This branch is based on `f431c0b256067daae7c154b7f62fd1e91004f891`

## Change Log

- 2021-05-19

change parameter `GOLF` to `GF`, add some function to debug

- 2021-05-14

fix bug that can not save parameter

add filter to AOA data

- 2021-05-11

Add NoopLoop AOA Drivers

- 2021-07-03

Add NoopLoop TOF Drivers

2021-08-15

merge in Rover 4.1

## Change Files

In `APMrover2` folder the following files has been changed from `f431c0b256067daae7c154b7f62fd1e91004f891`

```
# macro or head define and some function declaration
defines.h
system.cpp
Rover.cpp
Rover.h 


# add some mavlink msg to ctl
GCS_Mavlink.cpp

# modify somthing to add mode
mode.cpp
mode.h
mode_auto.cpp
mode_go_batt.cpp
mode_manual.cpp

# add some parameters
Parameters.cpp
Parameters.h



# function to implement task
Rover_golf.cpp
```



## Function Descriptions

### NoopLoop TOF

To use NoopLoop TOF Driver BrightSoul developed, you need follow settings below:

- set Rngfnd2_type to 40 (Which set in `libraries\AP_RangeFinder\RangeFinder.h`)
- set Rngfnd2_NUM to how many NoopLoop TOF in use
- set SERIAL4_BAUD to 921 to set SERIAL baud rate to 921600
- set SERIAL4_PROTOCOL to 9 to enable the Range Finder protocol

With NoopLoop TOF In series using, get data in Class rover like example below:

```
float test_distance_cm;
test_distance_cm = rangefinder.get_data((uint8_t)0);
gcs().send_text(MAV_SEVERITY_INFO, "0 %f", test_distance_cm); 
test_distance_cm = rangefinder.get_data((uint8_t)1);
gcs().send_text(MAV_SEVERITY_INFO, "1 %f", test_distance_cm); 
```

`rangefinder.get_data((uint8_t)0);`  means get data form NoopLoop TOF ID=0.

### NoopLoop AOA

To use this Driver BrightSoul developed, you need follow settings below:

- set BCN_TYPE to 15 (means using Nooploop AOA Driver BrightSoul dev version)
- set SERIAL4_BAUD to 921 to set SERIAL1’s baud rate to 921600
- set SERIAL4_PROTOCOL to 13 to enable the Beacon  protocol

> Note: you can also set Serialx to what you exact using

When message send like `AOA 0.86 4.85`, the AOA connect and work successful.

### Rover Guide to Stick

When switch to `case 9000` it will turn to yaw 0 (North). Then it will guide to stick follow the formula 


```
vel 		=   GF_GD_AHEAD
steer_rate 	= 	GF_GD_YAWRATE * angel_from_AOA
```

it recommed set `GF_GD_YAWRATE` from around `50~300`

