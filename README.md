# STM32-Bluetooth-Based-Home-Automation

Used STM32 microcontroller and various sensor to collect data and control the led lights , dc motor and send the data to a user mobile and controlled the outputs through bluetooth

* Bluetooth module used HC-05 

1. Bluetooth is IEEE 802.15.1 standardized protocol, through which one can build wireless Personal Area Network (PAN). It uses frequency-hopping spread spectrum (FHSS) radio technology to send data over air.

2. Communication between bluetooth and microcontroller is done using UART (Universal Asynchronous Receiver and Transmitter) protocol

3.The UART packet size is about 8 bits , 1 start and 1 stop bit and 0 parity bits.

* Sensors Used
1. LM-35 -> temperature sensor 
2. LDR -> Light brightness in a room
3. PIR -> To detect presence of someone inside a room

* Outputs

1. Based on sensing outputs were produced in the form of PWM signal using timmer 16 and 17 from main.c file. 

2. Based on temperature , if it's high, then speed of fan (dc motor) was increased  and decreased if it's low by adjusting the pwm signal.

3. Based on the intensity of light the brightness of LED lights was adjusted.
