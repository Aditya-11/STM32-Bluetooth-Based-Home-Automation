# STM32-Bluetooth-Based-Home-Automation

* Used ARM-Keil IDE

<a href="https://github.com/Aditya-11/STM32-Bluetooth-Based-Home-Automation/blob/master/LICENSE"><img src="https://img.shields.io/badge/license-MIT-Blue.svg" alt="License" /></a>

* HC-05 module

<a><img src="https://core-electronics.com.au/media/catalog/product/cache/1/image/650x650/fe1bcd18654db18f328c2faaaf3c690a/0/1/018-hc-05.jpg"></img></a>

# STM32f303RE microcontroller was used in the project.

Used STM32 microcontroller and various sensor to collect data and control the led lights , dc motor and send the data to a user mobile and controlled the outputs through bluetooth

# Bluetooth module used HC-05 

1. Bluetooth is IEEE 802.15.1 standardized protocol, through which one can build wireless Personal Area Network (PAN). It uses frequency-hopping spread spectrum (FHSS) radio technology to send data over air.

2. Communication between bluetooth and microcontroller is done using UART (Universal Asynchronous Receiver and Transmitter)   protocol

3. The UART packet size is about 8 bits , 1 start and 1 stop bit and 0 parity bits.

# Sensors Used

1. LM-35 -> temperature sensor 
2. LDR -> Light brightness in a room
3. PIR -> To detect presence of someone inside a room

# Outputs

* 1st to control the light intensity , 2nd -> To drive a dc motor (fan)

1. Based on sensing data outputs were produced in the form of PWM signal using timmer 16 and 17 from main.c file. 

2. Based on temperature , if it's high, then speed of fan (dc motor) was increased  and decreased if it's low by adjusting the pwm signal.

3. Based on the intensity of light the brightness of LED lights was adjusted by adjusting the PWM values.

4. The values of temperature , light intensity % , presence of a person was send to a persons mobile through bluetooth module.
And from the bluetooth app person had the option to control speed of fan. This was done by sending by sensing data through blutooth from mobile to blutooth module therough bluetooth protocol and from there through UART to the microcontroller. Based on the data the dutycycle of the PWM waveform was varied to control the speed of the motor.
