// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub.Bank;

/**
 * A servo class for servos plugged into a REV ServoHub for Continuous Rotation (CR) mode.
 * 
 * The ServoHub must be initialized before using this class.
 * A ServoHub has 6 ports for servos and is addressable by CAN ID.
 * 
 * Within the ServoHub, each Servo is addressed by a ChannelId.
 */
public class CRHubServo implements BaseServo {

    private ServoHub hub;
    ServoChannel channel;

    // Pulse width values for the servo as per ServoHub docs for CR mode
    private static final int SERVO_CCW_PULSE_WIDTH = 500;
    private static final int SERVO_CW_PULSE_WIDTH = 2500;
    private static final int SERVO_STOP_PULSE_WIDTH = 1500;

    public CRHubServo(ServoHub hub, int servoPortNum) {
        this.hub = hub;
        hub.setBankPulsePeriod(Bank.kBank3_5, 5000);
        setServo(servoPortNum);
    }

    private void setServo(int servoPortNum) {
        switch (servoPortNum) {
            case 0:
                channel = hub.getServoChannel(ChannelId.kChannelId0);
                break;
            case 1:
                channel = hub.getServoChannel(ChannelId.kChannelId1);
                break;
            case 2:
                channel = hub.getServoChannel(ChannelId.kChannelId2);
                break;
            case 3:
                channel = hub.getServoChannel(ChannelId.kChannelId3);
                break;
            case 4:
                channel = hub.getServoChannel(ChannelId.kChannelId4);
                break;
            case 5:
                channel = hub.getServoChannel(ChannelId.kChannelId5);
                break;
        }
        channel.setEnabled(true);
        channel.setPowered(true);
    }

    @Override
    public ServoMode getMode() {
        return ServoMode.CONTINUOUS_ROTATION;
    }

    @Override
    public void turnCounterclockwise() {
        channel.setPulseWidth(SERVO_CCW_PULSE_WIDTH);
    }

    @Override
    public void turnClockwise() {
        channel.setPulseWidth(SERVO_CW_PULSE_WIDTH);
    }

    @Override
    public void stop() {
        channel.setPulseWidth(SERVO_STOP_PULSE_WIDTH);
    }

    @Override
    public void setPulseWidth(int pulseWidth) {
        channel.setPulseWidth(pulseWidth);

    }

    @Override
    public void setSpeed(double speed) {
        // Map -1.0 to 1.0 to pulse width range
        int pulseWidth = (int)(SERVO_STOP_PULSE_WIDTH + (speed * (SERVO_CW_PULSE_WIDTH - SERVO_STOP_PULSE_WIDTH)));
        channel.setPulseWidth(pulseWidth);
    }

    @Override
    public double getCurrent() {
        return channel.getCurrent();
    }
}