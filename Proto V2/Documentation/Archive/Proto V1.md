---
tags:
  - note
  - archive
---
```table-of-contents
```

# What was [Proto V1](Archive/Proto%20V1.md)?
Proto V1 was the first rendition of the Proto(type) Robotics Platform here at the TWU RRU.

(picture goes here)

Proto V1 was the first ever robot built at TWU, and from scratch on top of that! It was a pioneer project with ambitious goals to build a fully autonomous book deliver robot in just 4 months over the summer semester.
The project did not succeed in its original goals, but through it the RRU was able to accumulate a lot of experience and knowledge that is now contained in [this documentation](Welcome.md).
Proto V1 at completion by 2024-08-30 was a teleoperated robot controlled through a web application written by [Oliver (LinYun) Liu](People/Oliver%20(LinYun)%20Liu.md).

# Systems Overview
- [Motor Control System](Archive/Proto%20V1%20Motor%20Control%20System%20Overview.md)
- [Power Supply System](Archive/Proto%20V1%20Power%20Supply%20System.md)
- Control System is just Raspberry Pi 4, ROS2 (with Nav2 module), and [Python 3](https://www.python.org/) code run with [gpiozero](https://gpiozero.readthedocs.io/en/latest/) code.