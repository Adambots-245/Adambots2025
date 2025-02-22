In VS Code, right-click this file in the Explorer and then choose Preview to view it.

Files placed in the deploy directory will be deployed to the RoboRIO into the
'deploy' directory in the home folder. Use the 'Filesystem.getDeployDirectory' wpilib function
to get a proper path relative to the deploy directory.

# How to tune the JSON files for our robot

## Prepare the Robot
* First mount the robot on a cart ensuring that the wheels are not touching the sides
* Turn the Swerve Modules in such a way that the bevel gears are pointing to the left, when the robot's front is ahead of you.
![Robot Image](https://docs.yagsl.com/~gitbook/image?url=https%3A%2F%2F567506766-files.gitbook.io%2F%7E%2Ffiles%2Fv0%2Fb%2Fgitbook-x-prod.appspot.com%2Fo%2Fspaces%252F754c0Fpq8fBi6k4ByS1k%252Fuploads%252FP8kIHNspDzTMqBiHw1U1%252Fimage.png%3Falt%3Dmedia%26token%3D9923865e-47c7-4b1e-90a5-9da9610c0764&width=768&dpr=1&quality=100&sign=6a519e95&sv=2)

## Edit the Module JSON Files (backleft, backright, frontleft, frontright - deploy/swerve/kraken/modules/)
* Note and change the drive type (Kraken uses talonfx) - talonfx for drive and sparkmax_neo for angle
* Use Phoneix tuner (talonfx and cancoder) and Rev Robotics Tool(sparkmax_neo) to find the ID for the devices and set the CAN ID in the id field.
* Rotate the drive wheel CCW (moving “forward”) - Check in Phoenix tuner that the built-in encoder value is increasing. If not, invert the drive motor (set inverted = true).
* Rotate the entire swerve CCW (when viewed from the top) - Check in Rev Tool - The built-in encoder value should increase. If not, invert the angle motor.
* Measure the swerve module's center relative to the robot center. Use a measuring tape to find the horizontal (left/right) and vertical (front/back) distances from the robot's center to the center of each swerve module. Distances toward the front and left are positive; distances toward the back and right are negative. So, FL (X is +ve, Y is +ve), FR (+, -), BL (-, +), BR (-, -).
* If we are using a CANivore device to create a separate CAN bus, only then set the canbus proprty to "canivore". If not, leave it null. Typically, we don't use canivore for drivetrain.

## Absolute Encoder Offset
* Turn Robot On (Disabled so the wheels can be turned manually)
* Set the robot wheels as shown in the diagram above. Use a straight-edge to ensure that the wheels are all aligned straight.
* Launch Phoenix Tuner: Open the Phoenix Tuner application on your computer.
* Zero out all the absoluteEncoderOffset values in all the JSON files.
* Deploy the code to the robot and then open up shuffleboard. Locate the swerve/rawencodervalue in shuffleboard and note down the values for all the modules.
* Determine Desired Offset: Since the wheels are aligned to a known reference (e.g., facing forward), the goal is to set this position as zero degrees. 
* Set the values you found in the JSON files as the absoluteEncoderOffset values. Save and re-deploy. Now the shuffleboard values for the absolute encoder values should be near zero.

## Physical Properties (deploy/swerve/kraken/modules/physicalproperties.json)
- Angle Gear Ratio - The MK4i Swerve Module from Swerve Drive Specialties features a steering (angle) gear ratio of 150/7:1, which is approximately 21.43:1. This gear ratio remains constant across all configurations of the MK4i module, including when using different drive gear ratios such as L2 or L2+. A 21.43:1 gear ratio means that the motor must rotate approximately 21.43 times to produce one complete rotation of the steering mechanism.
- Angle Factor - This represents how many degrees does one rotation of the motor correspond to and is expressed in degrees per motor rotation. So the formula is - 360 (degrees) / (Gear Ratio * Encoder Resolution). YAGSL documentation states that other than for a few motors, consider the Encoder resolution as 1. Hence, the factor = 360 / 21.43 = 16.8.
- Drive Gear Ratio - For L2+ gear configuration, the ratio is 5.9:1.
- Wheel diameter in inches - measure the diameter - we typically use a 4 inch wheel. This is same as 0.1016 meters.
- Drive Factor - The drive conversion factor translates motor rotations into linear wheel travel distance, expressed in meters per rotation. When a wheel rotates, one rotation covers the same distance as the circumference of the wheel. Hence, the distance travelled per rotation is = Circumference / gear ratio. So, the formula is PI * 0.1016 meters / 5.9 = 0.0542.
- Current Limit - This parameter sets the maximum current (in amperes) that can be supplied to the motors, preventing them from drawing excessive current, which could lead to overheating or damage. Leave it to typical values of 40 amps for Drive and 20 ams for Angle. Do NOT exceed this without consulting with the electrical team.
- Ramp Rate - Specifies the minimum time (in seconds) for the motor to transition from 0 to full throttle, controlling the acceleration rate. A ramp rate of 0.25 seconds provides a balance between responsiveness and mechanical stress. It ensures smooth acceleration, reducing the risk of wheel slippage and mechanical wear, while maintaining adequate responsiveness for competitive scenarios. Adjust this if you see issues with how the motor is responding when you move the Joystick.
- Wheel Grip Coefficient of Friction - Represents the grip level of the robot's wheels on the playing surface, influencing traction and acceleration capabilities. Our carpets should mirror competition field and 1.19 is a common value used in FRC. Do NOT change this unless you know what you are doing.
- Optimal Voltage - FRC robots operate with a 12V electrical system. The sweve system can use this information to apply appropriate feedforward control to ensure consistent performance. Leave it at 12.

## PIDF properties (deploy/swerve/kraken/modules/pidfproperties.json)
PIDF Settings in pidfproperties.json:

- Drive Motor Control: These parameters manage the speed and torque of the drive motors, ensuring accurate translation movements.
- Angle Motor Control: These settings regulate the rotation of the swerve modules, allowing precise wheel orientation for effective maneuvering.
* Define the PIDF values
    - p (Proportional Gain): Determines the reaction to the current error. A higher p value increases the system's responsiveness but may induce oscillations if set too high.
    - i (Integral Gain): Addresses accumulated past errors. It's useful for eliminating steady-state errors but can cause instability if overemphasized.
    - d (Derivative Gain): Responds to the rate of change of the error, providing damping to the system. It helps reduce overshoot and oscillations.
    - f (Feedforward Gain): Predicts the necessary control output based on the desired setpoint, enhancing the system's responsiveness, especially in velocity control.
    - iz (Integral Zone): Defines a threshold for the integral term activation. The integral component is applied only when the error is within this zone, preventing integral windup.
* Tuning Parameters 
    - Drive Motors: Start with a small p value (e.g., 0.0020645) and adjust based on system response. Typically, i, d, and f values are set to zero initially and introduced as needed during fine-tuning.
    - Angle Motors: Begin with a moderate p value (e.g., 0.01). For motors like the TalonFX/Kraken/Falcon, higher p values (around 50) and d values (around 0.32) may be necessary due to their characteristics. Since we are using Neo, we can start with a smaller value.
    - Technically, we can do some level of tuning using simulation before we tune it further in the real field. Check out https://youtu.be/Lr3FMEXtFi4.

## Controller Properties 
Robot Orientation Control: This PID controller maintains and adjusts the robot's overall heading during movement, ensuring it faces the desired direction. While this is crucial for autonomous operations to follow predefined paths, it also enhances teleoperated control by stabilizing and correcting the robot's orientation based on joystick inputs.

In simple terms, the "heading" in the controller properties JSON file refers to how the robot rotates or points in a direction. The heading controller has three main components that control how precisely and smoothly the robot turns to face a desired direction:

- P (Proportional): How quickly the robot reacts to turn
- I (Integral): How well it corrects small, persistent turning errors
- D (Derivative): How smoothly it slows down as it approaches the target direction

Values:
* angleJoystickRadiusDeadband:  Sets the minimum radius for the angle control joystick to initiate heading adjustments. Start with 0.5.
* heading: Defines the PID (Proportional, Integral, Derivative) controller parameters for managing the robot's heading.

## Swervedrive Properties
This file defines various miscellaneous properties of the Swerve Drive.

* IMU - this defines the Gyro unit that we are using. Change the ID for the Pigeon IMU that we use by locating its CAN bus ID in Phoenix Tuner. If we use Canivore to create another CAN Bus, then change the canbus to "canivore". If not, leave it null.
* Modules - Refers to the JSON files for the Swerve Modules. Ensure it is in this order - FL, FR, BL, BR
* The Max Module Speed will vary based on FOC enabled or disabled. From the chart in https://www.swervedrivespecialties.com/collections/mk4i-parts/products/kit-adapter-16t-drive-pinion-gear-mk4i?variant=47576386502957, when Field Oriented Control (FOC) is enabled, the free speed is 17.1 and when disabled, it is 17.7. FOC is an advanced motor control algorithm that improves motor efficiency and control precision. So, I recommend enabling it. When FOC is enabled - 17.1 ft/s × 0.3048 m/ft = 5.21 m/s. When FOC is disabled - 17.7 ft/s × 0.3048 m/ft = 5.40 m/s. [ 1 foot = 0.3048 meters ]
* InvertedIMU flag - While the robot is on the cart, set it to false first. View the Robot's turn angle on the field in Shuffleboard. When you turn the cart counter clockwise, if the angle increases, keep inverted false. If not, change it to true.
Remember: Counter Clockwise rotation should result in a positive angle increase according to the standard coordinate system. If it doesn't, you need to invert the IMU. See the diagram above.
