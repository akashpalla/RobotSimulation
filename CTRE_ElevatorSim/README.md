# Elevator Simulation

 
**Goal: To Simulate an elevator subsystem for the 2018 game.** 
[Power Up Gameplay](https://www.youtube.com/watch?v=JDOd-OSOr60&feature=emb_logo) for those who are unfamiliar with the 2018 game. Elevators were used to place Cubes at different heights. 


**Code Requirements** 
The code for this simulation should be able to be deployed and work on a real robot without changing any code. You can use ```RobotBase.isreal()``` to check whether you are connected to an actual robot or running a simulation. 

The elevator has three setpoints: Bottom (0m), Middle (2m), Top (3m). Use buttons on Shuffleboard to move between the different setpoints. This can be done by by  ```SmartDashboard.putData("Button Name", new Command());```

Follow Command Based Structure, creating subsystem(s) and command(s) as needed

Use the ElevatorSim class as part of the WPILib Physics Simulation

* [Documentation](https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-simulation/index.html)
* [Example code](https://github.com/mcm001/allwpilib/tree/state-space-v2/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation)
 
Write the PID loop from scratch without using the WPI or CTRE libraries for PID Resources for PID:
1. https://youtu.be/jIKBWO7ps0w
2. https://youtu.be/Z24fSBVJeGs
3. https://blog.wesleyac.com/posts/intro-to-control-part-one-pid
 
 
 
Use Shuffleboard to plot your motor output, elevator position, buttons, and other information
 
Mess around with the gains for P, I, & D and see how changing each one affects your output 
 
![Shuffleboard](https://i.ibb.co/gDg9PDc/unnamed.png)

*Example plots on Shuffleboard*
 
**Additional Information:**
* Carriage Mass: 15 pounds
* Max Elevator Height: 11 feet
* Gear Ratio: 10:1 reduction
* Pulley Radius: 2 inches
* Elevator Motor: 2 775 pro’s controlled by 2 TalonSRX’s. (Use WPI_TalonSRX)
* Ticks/Revolution: 4096 (ticks are the unit of the encoder, 4096 encoder ticks = 1 rotation)

*Make sure to pay attention to units*
 
Create a branch of this repository with your name as the branch title. When you are finished with your code, make a commit titled “DONE”
 
**Optional**: Create a functional Limit Switch (you can manipulate its value through the Simulation GUI or create a wrapper that automatically changes the value based on the elevator position)
 
**Additional Projects:**
Here are some other optional projects you can work on with Robot Simulation. Feel free to explore other projects as well. [Chief Delphi](https://www.chiefdelphi.com/)  is a great resource to use. 
* Use WPILib’s motion profiling libraries to complete the Infinite Recharge at Home driving courses: https://www.chiefdelphi.com/t/infinite-recharge-at-home-autonav-challenges-using-wpilibs-drive-simulator/390485 
* Simulate the 2020 or any robot, simulating multiple subsystems
* Replace PID with motion profiling (Implementation: https://youtu.be/wqJ4tY0u6IQ) 
* Create a 1D motion profile generator 	
