// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** Add your docs here. */
public class CTREPneumaticSolenoid implements BaseSolenoid{
    private DoubleSolenoid solenoid;

    CTREPneumaticSolenoid(int forwardPort, int reversePort){
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, forwardPort, reversePort);
    }

    void forward(){
        solenoid.set(Value.kForward);
    }

    void reverse(){
        solenoid.set(Value.kReverse);
    }

    @Override
    public void enable() {
        forward();
    }

    @Override
    public void disable() {
        reverse();
    }

    @Override
    public void toggle() {
        set(!get());
    }

    @Override
    public boolean get() {
        return (solenoid.get() == Value.kForward);
    }

    @Override
    public void set(boolean value) {
        solenoid.set(value ? Value.kForward : Value.kReverse);
    }


}
