# RailRat

RailRat is a self propelled inspection device that will ride along standard-gauge (4 foot, 8.5 inches) railroad. It will scan both rails with a rotating 2D LiDAR to report geometry defects of varying severity, along with their GPS location. Reports will be networked to a central server. 

For the alpha build, LoRa will be used to send reports. In industrial deployments, RailRat will utilize the rail's networking system.


## Team Members
- James Chau - Rust firmware development.
- Joel Kubinsky Daughterboard & hardware integration.
- Noah Lunberry - Chassis and wheel design.
- Tony Pham - LiDAR integration.


## Hardware/Components Used
- STM32F439ZI Nucleo-144 development board.
- L298N motor driver.
