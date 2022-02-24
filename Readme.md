# CSE 316 Project: Room Environment Monitor

## [ Video Demonstration]( https://www.youtube.com/watch?v=MsSqF6tohxE&t=18s )

## A Brief Description

We will monitor the value of temperature, humidity and sound intensity of a room and notify the
manager when any of these parameters exceed a certain threshold.

## Required Equipments
-   ATmega32 Microcontroller
-   DHT22 Temperature and Humidity Sensor
-   FC-04 Sound Sensor
-   Sim900a GSM Module
-   LCD Display (16x2)
-   Arduino Uno R3
-   USBasp AVR Programmer

## How it works?

The DHT22 sensor sends 40 bits to the ATmega32. We parsed the binary number into two real numbers
representing humidity and temperature. The FC-04 sound sensor provides a voltage to the ATmega32
corresponding to the sound intensity. We converted the voltage to sound intensity in decibel. We
showed these parameters on the LCD display. After then, we checked if any of those parameters
crossed their respective critical value. If yes, we called the manager (so that immediate attention can
be given) and sent an SMS containing the current value of the parameter which crossed its critical
value through our GSM module.

