// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class NEOMotor implements BaseMotor{

    SparkMax motor;
    SparkMaxConfig config = new SparkMaxConfig();

    public NEOMotor(int portNum, boolean brushed){
        if(brushed){
            motor = new SparkMax(portNum, MotorType.kBrushed);
        }else{
           motor = new SparkMax(portNum, MotorType.kBrushless);
        }
    }

    @Override
    public void set(double speed){
        motor.set(speed);
    }

    @Override
    public void setInverted(boolean inverted){
        config.inverted(inverted);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setBrakeMode(boolean brake){
        if(brake){
            config.idleMode(IdleMode.kBrake);
        }else{
            config.idleMode(IdleMode.kCoast);  
        }

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public double getPosition(){
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");

    }

    @Override
    public double getVelocity(){
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");

    }

    @Override
    public double getAcceleration(){
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");

    }

    @Override
    public double getCurrentDraw(){
        return motor.getOutputCurrent();
    }

    @Override
    public void setPosition(double rotations) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }

    @Override
    public boolean getForwardLimitSwitch() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getForwardLimitSwitch'");
    }

    @Override
    public boolean getReverseLimitSwitch() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getReverseLimitSwitch'");
    }

    @Override
    public void enableVoltageCompensation(double value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'enableVoltageCompensation'");
    }

    @Override
    public void setStrictFollower(int deviceID) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setStrictFollower'");
    }
}
