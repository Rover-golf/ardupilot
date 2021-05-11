# ArduPilot Project

This branch is based on `f431c0b256067daae7c154b7f62fd1e91004f891`

## Change Log

- 2021-05-11

Add NoopLoop AOA Drivers





## Function Descriptions

### NoopLoop AOA

To use this Driver BrightSoul developed, you need follow settings below:

- set BCN_TYPE to 3 (means using Nooploop AOA Driver BrightSoul dev version)
- set SERIAL4_BAUD to 921 to set SERIAL1’s baud rate to 921600
- set SERIAL4_PROTOCOL to 13 to enable the Beacon  protocol

> Note: you can change Serial num to baud what you exact using

When message send like `AOA 0.86 4.85`, the AOA connect and work successful.

### Rover Guide to Stick

When switch to `case 9000` it will turn to yaw 0 (North). Then it will guide to stick follow the formula 


```
vel = \text{GOLF_GUIDE_FORWARD} \quad from -100 \to 100
steer_rate = GOLF_GUIDE_YAWRATE_K * angel_from_AOA
```

it recommed set `GOLF_GUIDE_YAWRATE_K` from around `100~300`