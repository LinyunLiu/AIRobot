---
tags:
  - note
  - archive
---
```table-of-contents
```

# What was [Proto V1](Archive/Proto%20V1/Proto%20V1.md)?
Proto V1 was the first rendition of the Proto(type) Robotics Platform here at the TWU RRU.


(need to add a picture here)


Proto V1 was the first ever robot built at TWU, and from scratch on top of that! It was a pioneer project with ambitious goals to build a fully autonomous book deliver robot in just 4 months over the summer semester.
The project did not succeed in its original goals, but through it the RRU was able to accumulate a lot of experience and knowledge that is now contained in [this documentation](Welcome.md).
Proto V1 at completion by 2024-08-30 was a teleoperated robot controlled through a web application written by [Oliver (LinYun) Liu](People/Oliver%20(LinYun)%20Liu.md) using motor control code written by [Jacobus Burger](People/Jacobus%20Burger.md).

# Systems Overview
- [Motor Control System](Archive/Proto%20V1/Proto%20V1%20Motor%20Control%20System%20Overview.md)
- [Power Supply System](Archive/Proto%20V1/Proto%20V1%20Power%20Supply%20System.md)
- Control System is just Raspberry Pi 4, ROS2 (with Nav2 module), and [Python 3](https://www.python.org/) code run with [gpiozero](https://gpiozero.readthedocs.io/en/latest/) code.

# Flaws

There were some flaws with with the system.
1. Because the BLDC motors were [ESC](https://www.tytorobotics.com/blogs/articles/what-is-an-esc-how-does-an-esc-work?srsltid=AfmBOopTrgME3qIPAx8bXEjr9je65uH3d2aY-UnsB7Ey80KPHAqlsR3q) controlled with the original [Proto V1 Motor Control System](Archive/Proto%20V1/Proto%20V1%20Motor%20Control%20System%20Overview.md), when the motors were deactivated upon arriving at a programmed position, or were deactivated in an emergency, there was nothing to maintain the position of the robot or stop it from rolling, which led to multiple failures in precision, navigation, and proper safety.
2. The majority of electronic circuitry was just used to wrangle/control the motors using the [Proto V1 Motor Control System](Archive/Proto%20V1/Proto%20V1%20Motor%20Control%20System%20Overview.md).
3. 