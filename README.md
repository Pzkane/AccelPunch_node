# AccelPunch_node
AccelPunch: 2 slave nodes for accelerometer reading from boxing gloves and punching bag

> Platform: NodeMCU 12-E (12-F)

## Gloves_master

Transmits reading from 2 _DFRobot_LIS2DW12_ accelerometers through WiFi socket (as a server)

## BoxingBag_master

Transmits reading from 1 _LIS3DHTR_ accelerometer through WiFi socket (as a server)

## Data Origin

Accelerometer reading (self)

## Data Receiver

[AccelPunch_application](https://github.com/Pzkane/AccelPunch_application)