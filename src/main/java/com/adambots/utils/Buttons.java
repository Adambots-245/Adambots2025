/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.utils;

import java.util.function.DoubleSupplier;

import com.adambots.RobotMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * All Game Controller Button Mappings
 */
public class Buttons {
        // Initialize controllers
        public static final CommandXboxController XboxController = new CommandXboxController(
                        RobotMap.kXboxControllerPort);
        public static final CommandJoystick ex3dPro = new CommandJoystick(RobotMap.kJoystickControllerPort);

        // Xbox Controller Buttons
        public static final Trigger XboxBackButton = XboxController.back();
        public static final Trigger XboxStartButton = XboxController.start();
        public static final Trigger XboxXButton = XboxController.x();
        public static final Trigger XboxYButton = XboxController.y();
        public static final Trigger XboxBButton = XboxController.b();
        public static final Trigger XboxAButton = XboxController.a();
        public static final Trigger XboxLeftBumper = XboxController.leftBumper();
        public static final Trigger XboxRightBumper = XboxController.rightBumper();
        public static final Trigger XboxLeftStickButton = XboxController.leftStick();
        public static final Trigger XboxRightStickButton = XboxController.rightStick();

        // Xbox Throttle triggers
        public static final Trigger XboxLeftTriggerButton = XboxController.leftTrigger(0.5);
        public static final Trigger XboxRightTriggerButton = XboxController.rightTrigger(0.5);

        // Xbox DPad
        public static final Trigger XboxDPadN = XboxController.pov(0);
        public static final Trigger XboxDPadNE = XboxController.pov(45);
        public static final Trigger XboxDPadE = XboxController.pov(90);
        public static final Trigger XboxDPadSE = XboxController.pov(135);
        public static final Trigger XboxDPadS = XboxController.pov(180);
        public static final Trigger XboxDPadSW = XboxController.pov(225);
        public static final Trigger XboxDPadW = XboxController.pov(270);
        public static final Trigger XboxDPadNW = XboxController.pov(315);

        // Joystick Buttons
        public static final Trigger JoystickButton1 = ex3dPro.button(1);
        public static final Trigger JoystickButton2 = ex3dPro.button(2);
        public static final Trigger JoystickButton3 = ex3dPro.button(3);
        public static final Trigger JoystickButton4 = ex3dPro.button(4);
        public static final Trigger JoystickButton5 = ex3dPro.button(5);
        public static final Trigger JoystickButton6 = ex3dPro.button(6);
        public static final Trigger JoystickButton7 = ex3dPro.button(7);
        public static final Trigger JoystickButton8 = ex3dPro.button(8);
        public static final Trigger JoystickButton9 = ex3dPro.button(9);
        public static final Trigger JoystickButton10 = ex3dPro.button(10);
        public static final Trigger JoystickButton11 = ex3dPro.button(11);
        public static final Trigger JoystickButton12 = ex3dPro.button(12);
        public static final Trigger JoystickButton13 = ex3dPro.button(13);
        public static final Trigger JoystickButton14 = ex3dPro.button(14);
        public static final Trigger JoystickButton15 = ex3dPro.button(15);
        public static final Trigger JoystickButton16 = ex3dPro.button(16);

        // Joystick Thumb Pad
        public static final Trigger JoystickThumbUp = ex3dPro.povUp();
        public static final Trigger JoystickThumbDown = ex3dPro.povDown();
        public static final Trigger JoystickThumbUpLeft = ex3dPro.povUpLeft();
        public static final Trigger JoystickThumbUpRight = ex3dPro.povUpRight();
        public static final Trigger JoystickThumbDownLeft = ex3dPro.povDownLeft();
        public static final Trigger JoystickThumbDownRight = ex3dPro.povDownRight();
        public static final Trigger JoystickThumbLeft = ex3dPro.povLeft();
        public static final Trigger JoystickThumbRight = ex3dPro.povRight();
        public static final Trigger JoystickThumbCenter = ex3dPro.povCenter();

        /**
         * Return a value only if it is greater than a threshold, otherwise return 0
         * <p>
         * DO NOT USE ON TOP OF applyCurve, ADJUST THE CURVE ITSELF TO HAVE DESIRED
         * DEADZONE
         */
        public static double deaden(double input, double deadenThreshold) {
                if (Math.abs(input) < deadenThreshold) {
                        return 0;
                } else {
                        return input;
                }
        }

        // public static class Curve{
        // Double[] lookupTable;

        // //Parses data string into lookupTable on initialization
        // public Curve(String data){
        // ArrayList<Double> vals = new ArrayList<Double>(); //Create empty ArrayList

        // String[] input = data.trim().split(","); //Split string by comma to get each
        // value
        // for (String str : input) {
        // if (str.contains("*")) {
        // vals.add(Double.valueOf(str.substring(0, str.length()-1))); //For each value
        // remove the marker if there is one and append the number to vals
        // } else {
        // vals.add(Double.valueOf(str));
        // }
        // }

        // this.lookupTable = vals.toArray(new Double[vals.size()]); //Convert vals
        // (ArrayList) to lookupTable (Double[]) for faster lookup time and ease of use
        // }

        // public double lookup(int index){
        // return lookupTable[index];
        // }
        // }

        // /**
        // Applies a custom curve to an input and returns the result
        // */
        // public static double applyCurve (double rawInput, Curve curve) {
        // return
        // curve.lookup((int)Math.floor(Math.abs(rawInput)*100))*Math.signum(rawInput);
        // }

        // /*
        // DO NOT ADJUST CURVES MANUALLY!!!
        // Go to Adambots-245/Utils/Curve_Creator and you can import these curves into
        // the program to adjust them, or make new ones

        // Basic data structure (GENREALLY DO NOT NEED TO WORRY ABOUT):
        // 0.0*, 0.01, 0.02, ... 1.0*
        // the lookup table is a list of y values, with the index being the x value
        // if there is a * next to a number, that means the point represented there is
        // one of the ones used to draw the curve itself (allows you to reimport curves
        // back into Curve_Creator)
        // note - using a precalculated lookup table allows for much faster execution
        // times, since no math is required to interpolate between points
        // */
        // public static Curve forwardCurve = new
        // Curve("0.0*,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*,0.0,0.01,0.01,0.02,0.02,0.03,0.04,0.04,0.05,0.05*,0.06,0.06,0.07,0.07,0.08,0.08,0.09,0.09,0.1,0.1,0.11,0.11,0.12,0.12,0.13,0.13,0.14,0.14,0.15,0.15,0.16,0.16,0.17,0.17,0.18,0.18,0.19,0.19,0.2,0.2*,0.21,0.22,0.23,0.24,0.25,0.26,0.27,0.28,0.29,0.3,0.31,0.32,0.33,0.34,0.35,0.36,0.37,0.38,0.39,0.4*,0.42,0.44,0.46,0.48,0.5,0.52,0.54,0.56,0.58,0.6*,0.62,0.64,0.66,0.68,0.7,0.72,0.74,0.76,0.78,0.8,0.82,0.84,0.86,0.88,0.9,0.92,0.94,0.96,0.98,1.0*");
        // public static Curve sidewaysCurve = new
        // Curve("0.0*,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*,0.0,0.01,0.01,0.02,0.02,0.03,0.04,0.04,0.05,0.05*,0.06,0.06,0.07,0.07,0.08,0.08,0.09,0.09,0.1,0.1,0.11,0.11,0.12,0.12,0.13,0.13,0.14,0.14,0.15,0.15,0.16,0.16,0.17,0.17,0.18,0.18,0.19,0.19,0.2,0.2*,0.21,0.22,0.23,0.24,0.25,0.26,0.27,0.28,0.29,0.3,0.31,0.32,0.33,0.34,0.35,0.36,0.37,0.38,0.39,0.4*,0.42,0.44,0.46,0.48,0.5,0.52,0.54,0.56,0.58,0.6*,0.62,0.64,0.66,0.68,0.7,0.72,0.74,0.76,0.78,0.8,0.82,0.84,0.86,0.88,0.9,0.92,0.94,0.96,0.98,1.0*");
        // public static Curve rotateCurve = new
        // Curve("0.0*,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*,0.0,0.01,0.01,0.02,0.02,0.02,0.03,0.03,0.04,0.04,0.04,0.05,0.05,0.06,0.06,0.06,0.07,0.07,0.08,0.08*,0.09,0.09,0.1,0.1,0.11,0.12,0.12,0.13,0.13,0.14,0.15,0.15,0.16,0.16,0.17,0.18,0.18,0.19,0.19,0.2*,0.21,0.22,0.23,0.24,0.25,0.26,0.27,0.28,0.29,0.3,0.31,0.32,0.33,0.34,0.35,0.36,0.37,0.38,0.39,0.4*,0.42,0.44,0.46,0.48,0.5,0.52,0.54,0.56,0.58,0.6,0.62,0.64,0.66,0.68,0.7,0.72,0.74,0.76,0.78,0.8,0.82,0.84,0.86,0.88,0.9,0.92,0.94,0.96,0.98,1.0*");

        // Sigmoid Curve to smooth input values. Use this if you want a smooth curve.
        // Mr. B - DO NOT REMOVE THIS FUNCTION EVEN IF IT IS NOT USED
        public static double smoothInput(double input) {
                // Adjust the parameter 'a' to control the steepness of the curve
                double a = 4.0;

                // Apply a sigmoid function to the input value
                double sigmoid = 1.0 / (1.0 + Math.exp(-a * input));

                // Apply a hyperbolic tangent function to the input value
                // double tanh = Math.tanh(a * input);

                // Map the output range from (0,1) to (minOutput, maxOutput)
                double output = MathUtil.clamp(sigmoid, -1, 1);
                // double output = tanh * (maxOutput - minOutput) + minOutput;

                return output;
        }

        // Cubic Curve - An alternative to Sigmoid curve. Use this if you want a more
        // aggressive curve.
        // Mr. B - DO NOT REMOVE THIS FUNCTION EVEN IF IT IS NOT USED
        public static double cubic(double input) {
                double tuneA = 0; // try different values. However, tuneA and tuneB should add up to 1
                double tuneB = 1;

                return (tuneA * input) + (tuneB * Math.pow(input, 3));
        }

        public static double applyCurve(double rawInput) {
                if (Math.abs(rawInput) < 0.05) { // Small deadband
                        return 0;
                }

                if (RobotBase.isSimulation()) {
                        return rawInput;
                }

                return smoothInput(rawInput);
        }

        // public static DoubleSupplier forwardSupplier = () ->
        // -applyCurve(ex3dPro.getY(), forwardCurve);
        // public static DoubleSupplier sidewaysSupplier = () ->
        // -applyCurve(ex3dPro.getX(), sidewaysCurve);
        // public static DoubleSupplier rotateSupplier = () ->
        // -applyCurve(ex3dPro.getZ(), rotateCurve);

        public static DoubleSupplier forwardSupplier = () -> {
                // System.out.println("Forward: " + ex3dPro.getY());
                return -applyCurve(ex3dPro.getY());
        };
        public static DoubleSupplier sidewaysSupplier = () ->{
                // System.out.println("Sideways: " + ex3dPro.getX());
                return -applyCurve(ex3dPro.getX());
        };
        public static DoubleSupplier rotateSupplier = () -> {
                // System.out.println("Rotate: " + ex3dPro.getZ());
                return -applyCurve(ex3dPro.getZ());
        };

        /**
         * Rumble the XBox Controller
         * 
         * @param controller    pass the Xbox controller to rumble
         * @param timeInMillis  how many milliseconds to rumble the controller - max
         *                      value is 5000
         * @param intensity0to1 how intense should the rumble be
         */
        public static void rumble(CommandXboxController controller, int timeInMillis, int intensity0to1) {
                var joy = controller.getHID();
                final int time = MathUtil.clamp(timeInMillis, 0, 5000);

                // Perform an async operation to avoid scheduler overruns
                Thread rumbleThread = new Thread(() -> {

                        long rumbleStartTime = System.currentTimeMillis();

                        while (System.currentTimeMillis() - rumbleStartTime <= time) {
                                joy.setRumble(RumbleType.kBothRumble, intensity0to1); // Rumble both sides of the
                                                                                      // controller
                        }

                        joy.setRumble(RumbleType.kBothRumble, 0);
                });

                rumbleThread.start();
        }
}
