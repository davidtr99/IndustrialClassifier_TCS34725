# Industrial Classifier - Sistemas Electrónicos - 3º GIERM
## Required Hardware

- MSP430G2553 Microcontroller
- TCS34725 colour sensor
- IR Sensor
- HC06 Bluetooth adapter
- RGB Led
- Servomotor
- Continuous rotation servomotor

## Functionalities

This code implements a robust industrial classifier using the required hardware and the microcontroller internal peperipherals. To test its functionality we built a prototype wich sorts candies according to its colour.

In our example, we classify 5 types of colours: orange, pink, blue, red, green. Other colours will be classified as defects (in this case, others will be brown and yellow).

The system can be teleoperated using an mobile phone with bluetooth connection and our android app.

The mechanical parts could get stuck, and the program will react to this, solving it propertly.

Whenever a candy is classfied, the microcontroller stores the data in its internal flash memory, so if the system is turned off, the data will be restored when we turn it on again.

## Demo
Click [here](https://youtu.be/xGbJat-NOs4) to see the demo video of the project.

![Protype Image](https://github.com/davidtr99/IndustrialClassifier_TCS34725/blob/main/prototype.jpg?raw=true)

## Autors
- David Tejero Ruiz
- Pedro Barba Lozano
