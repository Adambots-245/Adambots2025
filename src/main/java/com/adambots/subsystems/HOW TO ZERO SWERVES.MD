# How to initialize the SwerveDrives

There are two ways to do it:
- In the code
- Using Phoenix Tuner software

We have decided to use the Phoenix Tuner method to set the initial values.

In the Phoenix Tuner, connect to the Robot and then identify the four Cancoders. Using a straight edge, ensure that the wheels are pointing straight in the robot. Start with the left side and align the two wheels.

Open the configuration page of the selected CANcoder and on the left side of the screen, press the "zero cancoder" button (Shaped like a small white circle)

Ensure the offset is applied correctly by running the self-test and check that the wheel absolute angle is zero (or close to the nearest few thousandth)

If wheels are zero'd while spun around 180 deg, then the wheel will spin backwards relative to the rest of the wheels when trying to drive
To resolve this, either flip the DriveMotorReversed flag in constants for that wheel, or re-zero the wheel with it spun around 180 deg

IF this method fails - try it the old way described below



Select one of the Cancoders that are aligned and then in the Config, make the following changes:

- Set the Sensor Initialization Strategy to BootToAbsolute
- Set the Magnet Offset to 0

Now, save the results and switch to Self-Snapshot tab. Here, run a self-snapshot and record the "Absolute" position.

Navigate back to the Control tab and set this absolute value in the Magnet Offset with a negative sign. Save and perform this for all the Cancoders.

Test by turning the wheels to different directions and then enabling the Robot. It should return to straight position.



Whenever the robot swerve modules are changed, remember to perform this to reinitialize the values.