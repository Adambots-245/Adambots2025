// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub.Bank;

/** Add your docs here. */
public class HubServo implements BaseServo {

    // This class is not intended for use if using multiple REV ServoHubs.

    private ServoHub hub;
    ServoChannel channel;

    // When initializing a REV ServoHub for the first time
    public HubServo(ServoHub hub, int servoPortNum) {
        this.hub = hub;
        hub.setBankPulsePeriod(Bank.kBank3_5, 5000);
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

    // When using a REV ServoHub that is already initialized 
    public HubServo(int servoPortNum) {
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
    public void turnCounterclockwise() {
        channel.setPulseWidth(500);
    }

    @Override
    public void turnClockwise() {
        channel.setPulseWidth(2500);
    }

    @Override
    public void stop() {
        channel.setPulseWidth(1500);
    }

    @Override
    public void setPulseWidth(int pulseWidth) {
        channel.setPulseWidth(pulseWidth);

    }
}
