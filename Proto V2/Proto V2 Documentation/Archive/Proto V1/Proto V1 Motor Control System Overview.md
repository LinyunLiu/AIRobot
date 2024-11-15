---
tags:
  - "#note"
  - archive
created: 2024-09-20
authors:
  - Jacobus Burger
---
The Motor Control System is the majority of the breadboard work in [Proto V1](Archive/Proto%20V1/Proto%20V1.md). It was mainly just to control the activation of the motors, to set the speed and direction of the motors, and to enable a category 2 [e-stop](https://www.controldesign.com/safety/safety-components/article/21526010/standards-guide-the-use-of-e-stops) by disconnecting signal to the motors without powering off the RPi4 when the emergency stop is activated.

An overview of the entire [Proto V1](Archive/Proto%20V1/Proto%20V1.md) circuit diagram can be viewed below:
![Circuit Overview](Archive/Proto%20V1/2024-08-23%20final%20electronic%20circuit%20diagram.png)
_Circuit Overview: A complete overview of the electronics system in the [Proto V1](Archive/Proto%20V1/Proto%20V1.md) project. GPIO13 (signal control) is fed through the NC line of the emergency stop (thus connection is severed when e-stop is active) and into the ME activation inputs, left and right DACs controlled over I2C by RPI4 over SDA (green) and SCL (white) lines output 0-5V through MEs into Motor Control Units (MCU) signal input, thus controlling speed of Left and Right motors individually. GPIO19 and GPIO20 control DIR input of MCU's to control direction of each motor independently. A 3v input line goes to GPIO6 through the NO line of the emergency stop and is read by motor control software (and thus software is alerted to deactivate motors persistently after e-stop is activated, so depressing the e-stop button doesn't reactivate motors). Otherwise 3 voltage levels are used to power the different parts of the system._

Notice there is a sub-circuit called "ME" for "Motor Enable" which toggles activation of the motors on bootup and runtime. This is off by default for the sake of safety. That is because during bootup originally the motors would automatically go to FULL SPEED on turn on, _which is **dangerous**_. If tampering with the motor control system, be sure to follow [Procedures](Procedures.md) to ensure safety. It's likely the motors could turn on unprompted. The circuit diagram for the "ME" on closeup is:
![](Archive/Proto%20V1/2024-08-23%20motor%20enable%20circuit.png)
_Motor Enable Circuit: A relay is used to control the flow of the DAC's 5V signal input to the motor signal input, using an NPN transistor controlled by a GPIO pin (GPIO13)_

And an overview of the Motor Control System is provided below:
![Motor Control System Diagram](Archive/Proto%20V1/2024-07-29%20motor%20control%20system%20diagram%20(Jacobus).jpg)
_Motor Control System: An Overview_

Notice that in both the circuit overview and motor control system that GPIO pins are listening in on the linear hall effect sensors within both the Left and Right motors. This information is read as input and triggers interrupts in software to calculate the tachometric data of each motor, determining their speed, direction of rotation, encoded position, and other information. More of that information can be found in the `taco.py` file in the source code.

E-Stop behaviour for this Motor Control System works as follows:
- when inactive, GPIO13 (Motor Enable Pin) can carry an on signal to the ME circuits (signified with red LEDs turning on), enabling output from DAC's to reach motor driver PWM/Signal pins.
- when active, GPIO13 connection is disabled (and thus ME are disabled). Thus motors are unable to be turned back on until emergency stop button is pressed
- at the same time, the motor controller code in `vel_controller.py` has a thread listening for input on GPIO6. When the E-Stop is activated, this will read 1/True, which will trigger the code to disable the motor enable circuit. This way, even when the emergency button is released, the motors stay disabled and need to be manually re-enabled