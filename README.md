# RailRat
RailRat is an autonomous railroad inspection device designed to bridge the gap between track geometry cars and ATGMS systems. It will traverse along standard-gauge (4 foot, 8.5 inches) railroad, scanning the rails with a rotating 2D LiDAR.  Any anomalies will be reported, along with their GPS location and the severity of the defect(s). Reports will be networked to a central server in real time as defects are detected. 


## Team Members
- James Chau - Rust firmware development.
- Joel Kubinsky - Daughterboard and hardware integration.
- Noah Lunberry - Chassis and wheel design/assembly.
- Tony Pham - LiDAR integration and data processing.


## Project Architecture

### External Interface
- A GUI will be provided to RailRat's users. It will connect directly to the RailRat via an LTE-M cellular network.
- Users can submit a GeoJSON file to configure their train station(s) and railways.
- A main window will display these stations and railroads, as well as any anomalies on the railroads. Anomalies can be flagged as resolved and removed from the main window.
- There will be another window to display the results of past inspection runs. There will be an option to restore any anomalies to the main window, should a user falsely marked an unresolved issue as resolved.


### Persistent State
- For the alpha, all user data will be stored locally.
- The user's stations, railways, and inspection runs will be stored in CSV files.
- During an inspection run, as defects are found, they will be added to the CSV file dynamically.
- Each entry will contain the type of anomaly found, its severity, its latitude, longitude, and altitude data, and any other necessary data.
- A cloud server may be deployed for future builds.

### Internal Systems
- Data collection and processing will be done on the RailRat cart by the STM32.
- The STM32 will collect data from the peripherals, with the IMU and LiDAR being used for railroad anomaly detection.
- If an anomaly is detected, it will send the inspection data to the persistent state via the LTE-M cellular network.

### Communication
- LiDAR <-> STM32 over UART (115200 baud) using Slamtec's RPLIDAR protocol.
- IMU <-> STM32 over I2C
- GPS <-> STM32 over UART (NMEA).
- STM32 controls the motor driver with PWM (speed) and GPIO (direction). It reads the motor encoder to calculate distance traveled and current velocity.
- The firmware subsystems run as separate Embassy asynchronous tasks connected by channels. LiDAR spins use a double buffer: one fills while the other is processed.
- Cart <-> ground station over an LTE-M cellular network. The ground station writes defects to a CSV and can send a stop command.

### Information Handling
- Each defect is sent as a packet with an event ID, sequence number, timestamp, location (GPS + milepost), defect type, severity, measured value, and CRC.
- Defects are saved to the cart's microSD before sending. The ground station replies with an ACK; if none arrives, the cart resends.
- Full LiDAR scans are logged to microSD and copied off after each run.

### Integrity & Resilience
- **Faulty data:** LiDAR samples have built-in check bits, GPS sentences have checksums, and every radio packet has a CRC. Bad data is dropped.
- **Interference:** ACK + retry recovers lost packets, sequence numbers detect gaps, and duplicates are dropped by event ID.
- **Intrusion:** packets carry a HMAC (keyed hash) so fake or replayed packets are rejected.

*Status: LiDAR check-bit validation is in progress with the driver; the rest is planned.*

## Completed Work
- Mechanical assembly of the RailRat cart has been completed. The chassis has been built, the wheels have been printed and attached, and testing on real train tracks has been done.
- Motor driver functionality has been integrated. It and its code was verified with o-scope readings.
- The battery has been connected to the build, supplying 12V to the motor and motor driver.
- The battery, motor, motor driver, and mechanical cart were integrated to perform movement tests on train tracks.
- The LiDAR sensor's spin and data transmit functionalities have been verified via the vendor's provided app.


## Known Bugs
- There is no 5V supply for the STM32.
- The STM32 has not yet been integrated with the LiDAR sensor.



## Hardware/Components Used

### Navigation & Power Supply
- JGB37-520 motor with built-in encoders.
- L298N motor driver.
- Might Max 12V 7Ah battery.

### Microcontroller
- STM32F439ZI Nucleo-144 development board.

### Sensors & Daughterboard Components
- RPLIDAR A1M8 LiDAR sensor.
- IAM-20680 IMU.
- SAM-M10Q GPS with a SparkFun breakout.
- IME-M module - **TBD**
- MicroSD SPI breakout
