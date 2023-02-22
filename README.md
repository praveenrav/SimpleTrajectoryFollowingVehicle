# SimpleTrajectoryFollowingVehicle

This project served as the final project for the ME 4405: Fundamentals of Mechatronics class I had taken during my undergraduate studies at the Georgia Institute of Technology.

As the name suggests, the course's objectives were to teach students about the fundamental concepts of mechatronics and microcontrollers, which includes microcontroller design & programming, mechanical actuators, sensors, feedback control, system modeling, and embedded software development. The microcontroller used for the course was the Texas Instruments MSP432P401R microcontroller, which was programmed in C using Code Composer Studio.

For the final project, the other students and I were instructed to form groups of two and design, manufacture, and program any type of mechatronics system showcasing some form of PID control. As a result, my partner and I had decided to create a simple trajectory-following differential-drive robot that rotates by a certain angle and traverses a certain distance. PID control was used to control the robot's velocity at 0.1 m/s, while bang-coast-bang control was implemented to determine whether or not the robot had reached its final destination.  

Originally, we had intended to use distance sensors (which would communicate strictly using the I2C protocol) and to implement obstacle-avoidance algorithms while traveling to the final destination; however, as it is notoriously difficult/tedious to implement the I2C protocol MSP432 microcontroller, our team was unable to accomplish this task within the project's time limits.

The design and manufacturing consisted of both the mechanical and electrical components of the system, which included chassis design in SolidWorks and the subsequent manufacturing, determination of specifications for the hardware components (motors, driver circuits, sensors, power sources, etc.), and assembly of all hardware components and circuitry.

The programming aspect consisted of programming the actuation, sensing, odometry, trajectory-planning, trajectory-following, and feedback control operations performed by the microcontroller.

The .c file consists of the code uploaded to the microcontroller, while the brief .m file consists of the template MATLAB code used to analyze the effectiveness of certain proportional, integral, and derivative gain configurations for the PID controller. 

More information about the project can be found in my engineering portfolio.                
    

        
          
