// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;

/** Add your docs here. */
public class ElectricalSolenoid implements BaseSolenoid {

    // Electrical solenoids are typically connected to the Relay port on the Rio
    private Relay relay;

    public ElectricalSolenoid(int port){
        relay = new Relay(port);
    }

    @Override
    public void enable() {
        relay.set(Value.kOn);
    }

    @Override
    public void disable() {
        relay.set(Value.kOff);
    }

    @Override
    public void toggle() {
        set(!get());
    }

    @Override
    public boolean get() {
        return (relay.get() == Value.kOn);
    }

    @Override
    public void set(boolean value) {
        relay.set(value ? Value.kOn : Value.kOff);
    }
}
