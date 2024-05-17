# pico-bldc
RP2040-based BLDC motor controller that I designed for our 2024 [PiWars](https://piwars.org/) robot, Firebot.

It controls four BLDC motors with field-oriented control and provides an I2C 
interface so it can be controlled by the main processor of the robot.

I wrote up the build of the controller on our blog, from slow-and-janky in Python
through to controlling 4 motors from one core in C:

https://www.hardwarehacker.co.uk/blog/tag/bldc/
