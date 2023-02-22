#include "msp.h"
#include "driverlib.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stdlib.h"

#define PI 3.14159265358979323846 // Defines pi

// Motor and Encoder constants and variables
const float wheel_diameter = 0.08; // Wheel diameter in meters
const float wheel_dist = 0.0275; // Distance between the two wheels
const float ticks_per_rev = 3; // Encoder ticks per revolution of the motor shaft
const float gear_ratio = 150.58; // Gear ratio of the motor
volatile int encoder_angle_ticks = 0; // Creates an integer that indicates the current number of ticks the encoder has accumulated while the robot is rotating
volatile int encoder_dist_ticks = 0; // Creates an integer that indicates the current number of ticks the encoder has accumulated while the robot is translating
volatile int encoder_old_dist_ticks = 0; // Creates an integer that indicates the old number of ticks the encoder has accumulated while the robot is translating
volatile int is_rot = 0; // Creates a boolean variable that determines whether or not the robot is rotating
volatile int is_trans = 0; // Creates a boolean variable that determines whether or not the robot is translating

// Location and orientation constants and variables - the coordinate plane on which the robot is moving is incremented in terms of millimeters
volatile float x = 0; // x-coordinate of robot's position
volatile float y = 0; // y-coordinate of robot's position
volatile float abs_x = 0; // Absolute value of the x-coordinate of the robot's position
volatile float abs_y = 0; // Absolute value of the y-coordinate of the robot's position
const float x_final = 2.5; // x-coordinate of robot's intended final destination
const float y_final = 2.5; // y-coordinate of robot's intended final destination
const float abs_x_final = 2.5; // x-coordinate of robot's intended final destination
const float abs_y_final = 2.5; // y-coordinate of robot's intended final destination
volatile float dist_diff; // Distance between the robot's instantaneous location and the specified final destination
volatile float dist_diff_init; // Initial difference in distance between the robot's initial location (0,0) to the robot's intended final location
volatile double dist_diff_sqr; // Square of the distance difference
volatile float angle_rob = 0; // Robot's angle in degrees
volatile float angle_rob_rad = 0; // Robot's angle in radians
volatile float angle_diff; // Difference between the robot's current angle and the angle it's supposed to face
volatile double tang; // Contains the tangent of the angle that the robot needs to face
volatile float angle_dir; // Contains the intended angle that the robot needs to face
volatile float abs_angle_diff; // Contains the absolute value of the difference between the intended angle that the robot needs to face and the robot's actual angle
volatile int direction = 0; // Equal to 1 if the robot needs to move CW, -1 if the robot needs to move CCW

// PID Control constants and variables
volatile int obstNearby = 0; // Creates a boolean that determines whether or not an obstacle lies in the robot's path
const float K_p = 100; // Proportional control constant
const float K_d = 10; // Derivative control constant
const float K_i = 0.33; // Integral control constant
volatile int motor_speed = 300; // Initial motor speed

//The variables for the encoder velocity, acceleration, and position
volatile float error_v = 0; // Difference in the robot's actual velocity and intended velocity
volatile float old_v = 0; // Robot's previous velocity
volatile float new_v = 0; // Robot's current velocity
const float desired_v = 0.1; // Desired velocity used in the PID control
volatile float error_a = 0; // Robot's instantaneous acceleration
volatile float error_p = 0; // Robot's instantaneous change in position
volatile float old_error_p = 0; // Robot's previous change in position
volatile float old_error_v = 0; // Robot's previous velocity error
volatile float old_dist = 0; // Robot's previous accumulated distance
volatile float new_dist = 0; // Robot's current accumulated distance
volatile float angle_init = 0; // Robot's initial angle
volatile float tot_dist_traveled = 0; // Total distance traveled by the robot
volatile float dist_traveled = 0; // Distance traveled by the robot with each time step
const float time_step = 0.5; // Time step of the timer
volatile int prop_err = 0; // Proportional error
volatile int deriv_err = 0; // Derivative  error
volatile int integ_err = 0; // Integral error
volatile float counter = 0; // Counter to keep track of the current time

#define timerA_divider   TIMER_A_CLOCKSOURCE_DIVIDER_64   // Means counter is incremented at 3E+6/64 = 46875 Hz
#define timerA_period2   23437.5 // Allows the timer to occur at a frequency of 2 Hz

// Timer for moving the motor clockwise
const Timer_A_UpModeConfig upConfig_1 =
{
     TIMER_A_CLOCKSOURCE_SMCLK,      // Tie Timer A to SMCLK
     timerA_divider,     // Increment counter every 64 clock cycles
     timerA_period2,                          // Period of Timer A (this value placed in TAxCCR0)
     TIMER_A_TAIE_INTERRUPT_DISABLE,     // Disable Timer A rollover interrupt
     TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Enable Capture Compare interrupt
     TIMER_A_DO_CLEAR            // Clear counter upon initialization
};

// Methods for conversion
float abs_val(float num) // Absolute value method
{
    if(num < 0)
    {
        num = num * -1;
    }

    return num;
}

int distance2ticks(float distance)
{
    float num_revs = distance/(PI * wheel_diameter); // Calculates the number of revolutions the wheel turned
    float num_revs_shaft = num_revs * gear_ratio; // Calculates the number of revolutions the motor shaft turned
    int ticks = (int) num_revs_shaft * ticks_per_rev; // Calculates the number of ticks the encoder accumulated

    return ticks; // Returns the number of ticks
}

float ticks2distance(float ticks)
{
    float num_revs_shaft = ticks/ticks_per_rev; // Calculates the number of revolutions the motor shaft turned
    float num_revs = num_revs_shaft/gear_ratio; // Calculates the number of revolutions the wheel turned
    float distance = num_revs * PI * wheel_diameter; // Calculates the distance the robot traveled

    return distance; // Returns the distance the robot traveled
}

int angle2ticks(float angle)
{
    float circum = PI * wheel_dist; // Calculates the circumference of the circle around which the wheels travel as the robot rotates
    angle = (angle * 2 * PI)/360; // Converts the unit of the given input angle from degrees to radians
    float dist_wheel_travel = angle * circum; // Calculates the distance each wheel needs to travel in order to rotate by the given input angle
    int ticks = distance2ticks(dist_wheel_travel); // Calculates the number of ticks the encoder needs to increment by in order to rotate by the given input angle

    return ticks; // Returns the number of ticks
}

float ticks2angle(int ticks)
{
    float circum = PI * wheel_dist; // Calculates the circumference of the circle around which the wheels travel as the robot rotates
    float dist_wheel_travel = ticks2distance(ticks); // Calculates the distance that the wheels traveled as the robot rotates
    float angle = (dist_wheel_travel/circum); // Calculates the angle in radians that the robot rotates
    angle = angle * (360/(2 * PI)); // Converts the unit of the calculated angle from degrees to radians

    return angle; // Returns the robot's angle
}

// Stop moving
void stopMoving(void)
{
    is_rot = 0;  // Sets the rotation boolean to 0
    is_trans = 0;  // Sets the translation boolean to 0

    // Turns off the PWM signals to the motors:
    TA0CCR1 = 1000;
    TA0CCR2 = 1000;
    TA0CCR3 = 1000;
    TA0CCR4 = 1000;
}

// Methods for robot rotation

// Turn clockwise
void turnCW(void)
{
    is_rot = 1; // Sets the rotation boolean to 1

    // Turns both motors clockwise
    TA0CCR1 = 900;                            // PWM duty cycle for one Timer A module
    TA0CCR2 = 1000;                            // PwM duty cycle for another Timer A module
    TA0CCR3 = 900;                            // PWM duty cycle for one Timer A module
    TA0CCR4 = 1000;                            // PwM duty cycle for another Timer A module
}

// Turn counter-clockwise
void turnCCW(void)
{
    is_rot = 1; // Sets the rotation boolean to 1

    // Turns both motors counter-clockwise
    TA0CCR1 = 1000;                            // PWM duty cycle for one Timer A module
    TA0CCR2 = 900;                            // PwM duty cycle for another Timer A module
    TA0CCR3 = 1000;                            // PWM duty cycle for one Timer A module
    TA0CCR4 = 900;                            // PwM duty cycle for another Timer A module
}

void main(void)
{
    WDT_A_holdTimer(); // Stops the watchdog timer
    FPU_enableModule(); // Enables the floating point module

    int i;
    for(i = 0; i < 100000; i++){}; // Causes a delay between when the program starts and when the robot starts moving

    // Setting up the clock:
    unsigned int dcoFrequency = 3E+6; // Initializes the DCO Frequency
    CS_setDCOFrequency(dcoFrequency); // Sets the DCO Frequency
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); // Initializes the clock signal

    /************************************** PWM ****************************************************/
    // Setting up the PWM signals for Motor 1
    P2SEL0 |= 0x10 ;    // Set bit 4 of P2SEL0 to enable TA0.1 functionality on P2.4 (AIN1)
    P2SEL1 &= ~0x10 ;   // Clear bit 4 of P2SEL1 to enable TA0.1 functionality on P2.4 (AIN1)
    P2DIR |= 0x10 ;     // Set pin 2.4 as an output pin (AIN1)

    P2SEL0 |= 0x20 ;    // Set bit 4 of P2SEL0 to enable TA0.2 functionality on P2.5 (AIN2)
    P2SEL1 &= ~0x20 ;   // Clear bit 4 of P2SEL1 to enable TA0.2 functionality on P2.5 (AIN2)
    P2DIR |= 0x20;     // Set pin 2.5 as an output pin (AIN2)

    // Setting up the PWM signals for Motor 2
    P2SEL0 |= 0x40 ;    // Set bit 4 of P2SEL0 to enable TA0.3 functionality on P2.6 (BIN1)
    P2SEL1 &= ~0x40 ;   // Clear bit 4 of P2SEL1 to enable TA0.3 functionality on P2.6 (BIN1)
    P2DIR |= 0x40 ;     // Set pin 2.6 as an output pin (BIN1)

    P2SEL0 |= 0x80 ;    // Set bit 4 of P2SEL0 to enable TA0.4 functionality on P2.7 (BIN2)
    P2SEL1 &= ~0x80 ;   // Clear bit 4 of P2SEL1 to enable TA0.4 functionality on P2.7 (BIN2)
    P2DIR |= 0x80;     // Set pin 2.7 as an output pin (BIN2)

    // Configure PWM signal for period of 0.333 ms, (initial) duty cycle of 0%
    TA0CCR0 = 1000;                           // PWM Period
    TA0CCR1 = 1000;                            // PWM duty cycle for one Timer A module
    TA0CCR2 = 1000;                            // PwM duty cycle for another Timer A module
    TA0CCR3 = 1000;                            // PWM duty cycle for one Timer A module
    TA0CCR4 = 1000;                            // PwM duty cycle for another Timer A module
    TA0CCTL1 = OUTMOD_7;                      // Reset/set output mode for Pin 2.4 (AIN1)
    TA0CCTL2 = OUTMOD_7;                      // Reset/set output mode for Pin 2.5 (AIN2)
    TA0CCTL3 = OUTMOD_7;                      // Reset/set output mode for Pin 2.6 (BIN1)
    TA0CCTL4 = OUTMOD_7;                      // Reset/set output mode for Pin 2.7 (BIN1)
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;  // SMCLK, up mode, clear TAR

    /* Setting up the PID Timer A */
    Interrupt_disableMaster(); // Disables all interrupts in NVIC
    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig_1); // Configure Timer A1 using above structure

    /* Setting up Encoder 1 */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN7); // Sets the input pin for Encoder 1
    GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN7, GPIO_LOW_TO_HIGH_TRANSITION); // Setting the GPIO interrupt to occur each time the encoder turns 1 count
    GPIO_enableInterrupt(GPIO_PORT_P1,GPIO_PIN7); // Enables the GPIO interrupt
    Interrupt_enableInterrupt(INT_PORT1); // Enables the NVIC Interrupt
    Interrupt_setPriority(INT_PORT1, 1); // Sets the priority for the GPIO Interrupt
    Interrupt_enableMaster(); // Enables all interrupts

    /* Starting the motional aspect of the program */
    if(x_final == 0) // Determines if the input x-coordinate of the final position is equal to 0
    {
        // Indicates to the robot that it doesn't need to rotate
        angle_dir = 0;
        angle_diff = 0;
        abs_angle_diff = 0;
    }
    else if(y_final == 0) // Determines if the input y-coordinate of the final position is equal to 0
    {
        if(x_final > 0) // Determines if the robot needs to turn right
        {
            // Sets the intended rotation angle to 90 degrees
            angle_dir = 90;
            angle_diff = 90;
            abs_angle_diff = 90;
        }
        else if(x_final < 0) // Determines if the robot needs to turn left
        {
            // Sets the intended rotation angle to -90 degrees
            angle_dir = -90;
            angle_diff = -90;
            abs_angle_diff = 90;
        }
    }
    else
    {
        tang = (double) (x_final - x)/(y_final - y); // Calculates the tangent of the angle that the robot needs to face in order to travel to its final destination
        angle_dir = (float) atan(tang); // Calculates the angle the robot needs to face in order to travel to its final destination
        angle_dir = (angle_dir * 360)/(2 * PI); // Converts the above angle to degrees
        angle_diff = angle_dir - angle_rob; // Calculates the difference in angle between the robot's current angle and the angle that the robot needs to face in order to travel to its final destination
        abs_angle_diff = abs_val(angle_diff); // Calculates the absolute value of the angle difference

    }

    dist_diff_sqr = (double) ((x_final - x) * (x_final - x)) + ((y_final - y) * (y_final - y)); // Calculates the square of the distance between the robot's instantaneous location and the specified final destination
    dist_diff = sqrt(dist_diff_sqr); // Calculates the distance between the robot's instantaneous location and the specified final destination
    dist_diff_init = dist_diff; // Sets the initial distance the robot is away from its intended final location

    while(1)
    {

        if(abs_angle_diff > 0.8) // Determines if the robot is not at its intended angle within a tolerance of 0.8 degrees
        {
            if(angle_diff > 0) // Determines if the difference in angle is positive (meaning the robot needs to turn clockwise)
            {
                turnCW(); // Turns the robot clockwise
                direction = 1; // Sets the direction of movement to clockwise
            }
            else if(angle_diff < 0) // Determines if the difference in angle is negative (meaning the robot needs to turn counter-clockwise)
            {
                turnCCW(); // Turns the robot counter-clockwise
                direction = -1; // Sets the direction of movement to counter-clockwise
            }

            while(abs_angle_diff > 0.8)
            {
                angle_rob = direction * ticks2angle(encoder_angle_ticks); // Calculates the angle of the robot
                angle_diff = angle_dir - angle_rob; // Calculates the difference in angle between the robot's current angle and the angle that the robot needs to face in order to travel to its final destination
                abs_angle_diff = abs_val(angle_diff); // Calculates the absolute value of the angle difference
            }

            stopMoving(); // Stops the robot from moving
        }


        // Causes the robot to travel to its final destination
        if((dist_diff > -0.03)) // Determines if the robot is not at its final location within a tolerance of 3 cm
        {
            // PID Control
            is_trans = 1; // Sets the translation boolean to 1
            Interrupt_enableInterrupt(INT_TA1_0); // Re-enables the TimerA Interrupt in order for the motor to rotate back to its home position
            Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);  // Start Timer A
            Interrupt_setPriority(INT_TA1_0, 0); // Sets the priority for the Timer A1 Interrupt
            Interrupt_enableMaster(); // Enables all interrupts in NVIC

            while((dist_diff > -0.03)){}; // Allows the robot to travel to its final location within a tolerance of 3 cm
            stopMoving(); // Stops the robot from moving
            Interrupt_disableMaster(); // Disables all interrupts in NVIC
        }
    }
}

// Timer A1 Interrupt for PID Control
void TA1_0_IRQHandler(void)
{
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0); // Clears the Timer A1 interrupt
    counter = counter + time_step; // Increments the counter

    // In order to move straight, Motor 1 must move counter-clockwise while Motor 2 must move clockwise
    is_trans = 1; // Sets the translation state boolean to true

    // Moves the robot forward at stated duty cycle
    TA0CCR1 = 1000;
    TA0CCR2 = 1000 - motor_speed;
    TA0CCR3 = 1000 - motor_speed;
    TA0CCR4 = 1000;

    dist_traveled = ticks2distance(encoder_dist_ticks - encoder_old_dist_ticks); // Calculates the distance traveled within the previous time step
    tot_dist_traveled = tot_dist_traveled + ticks2distance(encoder_dist_ticks - encoder_old_dist_ticks); // Calculates the total distance traveled
    encoder_old_dist_ticks = encoder_dist_ticks; // Calculates the total number of ticks accumulated by the encoder from the previous time step

    angle_rob_rad = (angle_rob * 2 * PI)/360; // Calculates the instantaneous angle of the robot in radians
    x = x + (dist_traveled * (float) sin((double) angle_rob_rad)); // Calculates the instantaneous x-coordinate of the robot's position
    y = y + (dist_traveled * (float) cos((double) angle_rob_rad)); // Calculates the instantaneous y-coordinate of the robot's position
    abs_x = abs_val(x); // Calculates the absolute value of the instantaneous x-coordinate of the robot's position
    abs_y = abs_val(y); // Calculates the absolute value of the instantaneous y-coordinate of the robot's position

    dist_diff_sqr = (double) ((x_final - x) * (x_final - x)) + ((y_final - y) * (y_final - y)); // Calculates the square of the distance between the robot's instantaneous location and the specified final destination
    dist_diff = sqrt(dist_diff_sqr); // Calculates the distance between the robot's instantaneous location and the specified final destination

    // The following conditional statements are used to determine if the robot moves past its intended final destination.
    // If the robot does move past its final intended destination, the difference in distance is represented as a negative number
    if(x_final == 0)
    {
        if((abs_y) > (abs_y_final))
        {
            dist_diff = dist_diff * -1;
        }
    }
    else if(y_final == 0)
    {
        if((abs_x) > (abs_x_final))
        {
            dist_diff = dist_diff * -1;
        }
    }
    else
    {
        if((((abs_x) > (abs_x_final))) || ((abs_y) > (abs_y_final)))
        {
            dist_diff = dist_diff * -1;
        }
    }

    new_dist = tot_dist_traveled; // Updates the new distance traveled
    new_v = (new_dist - old_dist)/time_step; // Calculates the new velocity
    error_v = desired_v - new_v; // Calculates the difference between the new velocity and the desired velocity

    /* Anti-Windup Method */
    if(((motor_speed > 1000) || (motor_speed < 0)) && (error_v * error_p > 0))
    {
        error_p = old_error_p; // Clamps the integral error
    }
    else
    {
        error_p = old_error_p + ((error_v) * time_step); // Calculates the integral error between the new velocity and the desired velocity
    }

    error_a = (new_v - old_v)/time_step;

    prop_err = (int) (K_p * error_v); // Calculates the proportional error
    deriv_err = (int) (K_d * error_a); // Calculates the derivative error
    integ_err = (int) (K_i * error_p); // Calculates the integral error

    motor_speed = motor_speed + prop_err - deriv_err + integ_err; // Updates the new motor speed


    // Saturates the motor speed
    if(motor_speed > 1000) // Determines if the calculated duty cycle is greater than 100%
    {
        motor_speed = 1000; // Sets the higher limit of the duty cycle at 100%
    }
    else if(motor_speed < 0) // Determines if the calculated duty cycle is lower than 100%
    {
        motor_speed = 0; // Sets the lower limit of the duty cycle at 0%
    }

    old_error_p = error_p; // Updates the previous position error
    old_error_v = error_v; // Updates the previous velocity error
    old_v = new_v; // Updates the previous acceleration error
    old_dist = new_dist; // Updates the previous total distance traveled
}

// GPIO Interrupt Handler - this interrupt is activated each time the encoders on the robot rotates by at least one click
void PORT1_IRQHandler(void)
{
    if((is_trans == 1) & (is_rot == 0)) // Determines if the robot is translating
    {
        encoder_dist_ticks++; // Updates the accumulated encoder ticks for translation
    }
    else if((is_rot == 1) & (is_trans == 0)) // Determines if the robot is rotating
    {
        encoder_angle_ticks++; // Updates the accumulated encoder ticks for rotation
    }

    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN7); // Clears the GPIO Interrupt flag
}
