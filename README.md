# Rancilio Silvia espresso machine mod
## Introduction
This modification was inspired by PostModernDesign's mod found here: https://github.com/PostModernDesign/RancilioBrain
The project aim is to create a mod for the rancilio silvia espresso machine, enabling gravimetric extraction.
Gravimetric extraction is the use of mass as the control variable for extraction amount. Thus, scales will be fitted in the machines drip tray and an arduino will control the pump, shutting it of to achieve a predefined chosen target weight. 
## Hardware
The project uses an Arduino Nano Every as the microcontroller. A rotary encoder is used as the input method to adjust target weight and move between states.
Two loadcells are used to measure the weight of coffee out of the machine. These loadcells are input into HX711 amplifiers to create the signal read by the Arduino.
An SSD1306 OLED display is used to display state information.
##Software
The code is written in C++ for Arduino. Libraries ares used for OLED control and reading from HX711.

The device runs in 3 states:

1. Pre_extraction - Enable change of target weight and display previous extraction time and weight out


2. Extracting
Zero scales, start pump, display current weight and extractin time. Pump is then shut off when target weight minus a overshoot is reached. Overshoot used to counteract extra coffee coming through after pump shutoff.

3. Post_extraction - After a pause to let last coffee drip out, display final weight and extraction time

States are moved between via press of the rotary encoder button. If pressed during 
