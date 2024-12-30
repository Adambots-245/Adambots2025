package com.adambots.actuators;

public interface BaseMotor {

    void set(double speed);

    void setInverted(boolean inverted);

    void setBrakeMode(boolean brake);

    void setPosition(double rotations);

    void enableVoltageCompensation(double value);

    double getPosition();

    double getVelocity();

    double getAcceleration();

    double getCurrentDraw();

    boolean getForwardLimitSwitch();

    boolean getReverseLimitSwitch();

    void setStrictFollower(int deviceID);
}
