# CE3002-2104
SENSOR INTERFACING &amp; DIGITAL CONTROL (SC2104/3002)

Using STM32F407VETX board version D

## Lab 1
Configuring OLED, UART, ultrasonic sensor

## Lab 2
Using 6-axis IMU - (accelerometer and gyroscope)
- sensor fusion to get better readings for pitch and roll angles (complementary filter)
- serial plot is used to fine tune the calculations
> note: unable to use accelerometer for yaw readings as it will give 9.8 by default due to gravitational acceleration 

## Lab 3
Using 6-axis IMU - (accelerometer and gyroscope)
- sensor fusion to get better readings for pitch and roll angles (complementary filter + kalman filter)
- serial plot is used to fine tune the calculations
> note: unable to use accelerometer for yaw readings as it will give 9.8 by default due to gravitational acceleration

## Lab 4
Using control theory to set up a PID control loop to correct a servo motor that turns to a specified angle
- serial plot is used to fine tune the calculations

## Lab 5
Using PID control to create a self-levelling platform, with pitch and roll readings
- serial plot is used to fine tune the calculations