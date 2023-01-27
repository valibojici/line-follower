# Line follower

## Description
This project was made as part of the Introduction to Robotics course.

## Task
This is a line follower project that was done in a team, [@Daria602](https://github.com/Daria602) ([Daria's repo](https://github.com/Daria602/LineFollower)) and I received a kit containing all the required components. After we assembled it we had 10 hours to program it and to complete a course in less than 20s for maximum points. The robot uses a microcontroller and reflectance sensors to follow black lines on a white surface. 

The robot is capable of following straight and curved lines and it calibrates itself when it starts. We used a PD controller to control the robot's movement and keep it on the line.

We completed the course in 20s.

## Demo
[Youtube link](https://youtu.be/GUKyuXd97sc)


## Components List
1. Arduino Uno
2. Zip-ties
3. Power source (can be of different shape). In our case, a LiPo battery
4. Wheels (2)
5. Wires for the line sensor (female - male)
6. QTR-8A reflectance sensor, along with screws
7. Ball caster
8. Extra wires from the kit or lab
9. Chassis
10. Breadboard - medium (400pts)
11. L293D motor driver
12. DC motors (2)

## Wire connections
1. Connect the QTR8-A (sensors 2-7) to A0-A5 pins
2. Connect the QTR8-A VIN and GND to the 5V and GND column (aka the one from
Arduino, NOT the one from the battery)
3. Connect the L293D driver to Arduino and to both motors
4. Connect the battery to VIN (understand the concept, first)
![image](https://user-images.githubusercontent.com/68808448/212927511-4d68ecff-2055-4d5c-9daa-92897d904884.png)


