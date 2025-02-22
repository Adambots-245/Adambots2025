package com.adambots.actuators;

import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub.Bank;

/**
 * A servo class for servos plugged into a REV ServoHub for Angular mode.
 * On Servos like Axon Max+, it is called Servo mode and needs to be set in the Axon Programmer tool.
 * 
 * On Axon Max+, the maximum angle is 355 degrees, but can be configured in the programmer tool.
 */
public class AngularHubServo implements BaseServo {
    private ServoHub hub;
    private ServoChannel channel;

    // Pulse width values for angular mode
    private static final int MIN_PULSE_WIDTH = 500;  // 0 degrees
    private static final int MAX_PULSE_WIDTH = 2500; // Max degrees (355 for Axon Max+)
    private static final int CENTER_PULSE_WIDTH = 1500;
    
    private final double maxAngle;

    /**
     * Creates a new AngularHubServo
     * @param hub The ServoHub this servo is connected to
     * @param servoPortNum The port number (0-5) on the ServoHub
     * @param maxAngle The maximum angle in degrees (typically 355 for Axon Max+)
     */
    public AngularHubServo(ServoHub hub, int servoPortNum, double maxAngle) {
        this.hub = hub;
        this.maxAngle = maxAngle;
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
        return ServoMode.ANGULAR;
    }

    @Override
    public void turnCounterclockwise() {
        channel.setPulseWidth(MAX_PULSE_WIDTH);
    }

    @Override
    public void turnClockwise() {
        channel.setPulseWidth(MIN_PULSE_WIDTH);
    }

    @Override
    public void stop() {
        channel.setPulseWidth(CENTER_PULSE_WIDTH);
    }

    @Override
    public void setPulseWidth(int pulseWidth) {
        channel.setPulseWidth(pulseWidth);
    }

    @Override
    public double getCurrent() {
        return channel.getCurrent();
    }

    @Override
    public void setAngle(double degrees) {
        // Clamp to valid range
        degrees = Math.min(maxAngle, Math.max(0, degrees));
        
        // Map angle to pulse width
        double normalizedPosition = degrees / maxAngle;
        int pulseWidth = (int)(MIN_PULSE_WIDTH + (normalizedPosition * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)));
        channel.setPulseWidth(pulseWidth);
    }
}