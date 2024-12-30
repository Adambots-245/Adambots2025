// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.DriveConstants.ModulePosition;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.ModuleConstants;
import com.adambots.sensors.BaseAbsoluteEncoder;
import com.adambots.sensors.CANCoder;
import com.adambots.utils.Dash;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;

@SuppressWarnings("unused") // Gets rid of warning when m_position is unused, as m_position is still useful
                            // to have when debugging/testing
public class SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final BaseAbsoluteEncoder m_turningEncoder;

  private final PIDController m_turningPIDController = new PIDController(ModuleConstants.kPModuleTurningController, 0,
      ModuleConstants.kDModuleTurningController);

  private ModulePosition m_position;

  /**
   * Constructs a SwerveModule.
   *
   * @param position              The position of this module (front or back,
   *                              right or left)
   * @param driveMotorChannel     The channel of the drive motor.
   * @param turningMotorChannel   The channel of the turning motor.
   * @param turningEncoderChannel The channels of the turning encoder.
   * @param driveMotorReversed    Whether the drive motor is reversed.
   */
  public SwerveModule(ModulePosition position, int driveMotorChannel, int turningMotorChannel,
      int turningEncoderChannel, boolean driveMotorReversed) {

    this.m_position = position; // Use position.name() to get the name of the position as a String

    // Create and configure the drive motor
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.idleMode(IdleMode.kBrake);
    driveConfig.smartCurrentLimit(ModuleConstants.kDriveCurrentLimit);
    driveConfig.voltageCompensation(ModuleConstants.kNominalVoltage);
    driveConfig.inverted(driveMotorReversed);
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create and configure the turning motor
    SparkMaxConfig turningConfig = new SparkMaxConfig();
    turningConfig.idleMode(IdleMode.kBrake);
    turningConfig.smartCurrentLimit(ModuleConstants.kTurningCurrentLimit);
    turningConfig.voltageCompensation(ModuleConstants.kNominalVoltage);
    turningConfig.inverted(true);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    // m_driveMotor.setIdleMode(IdleMode.kBrake);
    // m_driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveCurrentLimit);
    // m_driveMotor.enableVoltageCompensation(ModuleConstants.kNominalVoltage);
    // m_driveMotor.setInverted(driveMotorReversed);

    m_driveEncoder = m_driveMotor.getEncoder();

    // m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    // m_turningMotor.setIdleMode(IdleMode.kBrake);
    // m_turningMotor.setSmartCurrentLimit(ModuleConstants.kTurningCurrentLimit);
    // m_turningMotor.enableVoltageCompensation(ModuleConstants.kNominalVoltage);
    // m_turningMotor.setInverted(true);

    m_turningEncoder = new CANCoder(turningEncoderChannel);

    // Dash.add(m_position.name(), () ->
    // m_turningEncoder.getAbsolutePositionDegrees());
    // Dash.add("W Speed: " + m_position.name(), () ->
    // m_driveEncoder.getVelocity()*ModuleConstants.kDriveEncoderVelocityConversionFactor);
    // Dash.add("W Current: " + m_position.name(), () ->
    // m_driveMotor.getOutputCurrent());

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    resetDriveEncoders();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double speedMetersPerSecond = m_driveEncoder.getVelocity() * ModuleConstants.kDriveEncoderVelocityConversionFactor;
    Rotation2d turnAngleRadians = m_turningEncoder.getAbsolutePositionRotation2D();

    return new SwerveModuleState(speedMetersPerSecond, turnAngleRadians);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    double distance = m_driveEncoder.getPosition() * ModuleConstants.kDriveEncoderPositionConversionFactor;
    Rotation2d turnAngleRadians = m_turningEncoder.getAbsolutePositionRotation2D();

    return new SwerveModulePosition(distance, turnAngleRadians);
  }

  /**
   * Sets the desired state for the module.
   * 
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    double currentTurnAngle = m_turningEncoder.getAbsolutePositionRadians();

    // Optimize the reference state to avoid spinning further than 90 degrees
    // desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(currentTurnAngle));
    desiredState.optimize(new Rotation2d(currentTurnAngle));

    // Calculate the drive output by scaling the desired speed from +-max speed to
    // +-1, assuming constant resistive force, this works well.
    double driveOutput = desiredState.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;

    // Calculate the turning motor output from the turning PID controller.
    double turnOutput = m_turningPIDController.calculate(currentTurnAngle, desiredState.angle.getRadians());

    // Set the motors to the calculated outputs
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Sets a motor command of 0 */
  public void stopMotors() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }

  /** Zeroes the SwerveModule drive encoders */
  public void resetDriveEncoders() {
    m_driveEncoder.setPosition(0);
  }
}