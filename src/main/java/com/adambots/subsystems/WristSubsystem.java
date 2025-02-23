package com.adambots.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import com.adambots.actuators.BaseMotor;
import com.adambots.sensors.BaseAbsoluteEncoder;
import com.adambots.utils.StateMachine;
import com.adambots.RobotMap;
import com.adambots.Constants.ElevatorConstants;

public class WristSubsystem extends SubsystemBase {

  // Properties for Wrist State
  public record WristProperties(
      double angleDegrees,
      String description) {
  }

  // Define wrist states
  public enum WristState {
    STOWED(new WristProperties(ElevatorConstants.kWristStowedPosition, "Stowed")),
    INTAKE(new WristProperties(ElevatorConstants.kWristIntakePosition, "Intake Ready")),
    L1(new WristProperties(ElevatorConstants.kWristL1Position, "Level 1 Angle")),
    L2(new WristProperties(ElevatorConstants.kWristL2Position, "Level 2 Angle")),
    L3(new WristProperties(ElevatorConstants.kWristL3Position, "Level 3 Angle")),
    L4(new WristProperties(ElevatorConstants.kWristL4Position, "Level 4 Angle"));

    public final WristProperties properties;

    WristState(WristProperties properties) {
      this.properties = properties;
    }
  }

  // Hardware
  private final BaseAbsoluteEncoder wristEncoder;
  private final PIDController wristPID;
  private final BaseMotor wristMotor;

  double wristSpeed;

  // State Machine
  private final StateMachine<WristState, WristProperties> wristStateMachine;

  public WristSubsystem(BaseMotor wristMotor, BaseAbsoluteEncoder wristEncoder) {
    this.wristMotor = wristMotor;
    this.wristEncoder = wristEncoder;

    // Initialize PID
    this.wristPID = new PIDController(ElevatorConstants.kPWristController, ElevatorConstants.kIWristController,
        ElevatorConstants.kDWristController);
    this.wristPID.setTolerance(ElevatorConstants.kWristPositionTolerance);

    configureMotors();

    // Initialize wrist state machine without position control
    wristStateMachine = new StateMachine<>(
        WristState.STOWED,
        WristState.STOWED.properties,
        message -> SmartDashboard.putString("Wrist/Status", message),
        false // Not using position control
    );
  }

  private void configureMotors() {
    // Configure wrist motor
    wristMotor.setBrakeMode(true);
    // wristMotor.setInverted(true);
  }

  private void setWristOutput(WristProperties properties) {
    wristSpeed = wristPID.calculate(wristEncoder.getAbsolutePositionDegrees(),
        properties.angleDegrees());
    if (wristPID.atSetpoint()) {
      wristSpeed = 0;
    }
  }

  private boolean isWristAtTarget() {
    return wristPID.atSetpoint();
  }

  public void setWristSpeed(double speed) {
    wristSpeed = speed;
  }

  @Override
  public void periodic() {

    checkFailSafes();
    wristMotor.set(wristSpeed);

    // Get current position
    double currentAngle = wristEncoder.getAbsolutePositionDegrees();

    // Update wrist state machine
    wristStateMachine.periodic();

    // Update dashboard
    SmartDashboard.putNumber("Wrist/CurrentAngle", currentAngle);
    SmartDashboard.putNumber("Wrist/TargetAngle",
        wristStateMachine.getTargetProperties().angleDegrees());
    SmartDashboard.putString("Wrist/State",
        wristStateMachine.getCurrentState().toString());
    SmartDashboard.putBoolean("Wrist/AtTarget", isWristAtTarget());
  }

  // Public methods for commanding the wrist
  public void moveWristToState(WristState state) {
    wristStateMachine.requestTransition(
        state,
        state.properties,
        this::isWristAtTarget,
        this::setWristOutput);
  }

  // Methods to check states
  public boolean isWristAtState(WristState state) {
    return wristStateMachine.getCurrentState() == state && isWristAtTarget();
  }

  public WristState getCurrentWristState() {
    return wristStateMachine.getCurrentState();
  }

  public void checkFailSafes() {

    double elevatorCurrentHeight = RobotMap.elevatorMotor.getPosition() * ElevatorConstants.kInchesPerRotation;
    
    if (wristSpeed > 0 && wristEncoder.getAbsolutePositionDegrees() >= ElevatorConstants.kWristMaxAngle) {
      wristSpeed = 0;
    }
    if (wristSpeed < 0 && wristEncoder.getAbsolutePositionDegrees() <= ElevatorConstants.kWristMinAngle) {
      wristSpeed = 0;
    }
    if (elevatorCurrentHeight >= ElevatorConstants.kElevatorDangerZoneStart
        && elevatorCurrentHeight <= ElevatorConstants.kElevatorDangerZoneEnd) {

      if (wristEncoder.getAbsolutePositionDegrees() >= ElevatorConstants.kWristDangerZoneAngle) {

        if (wristSpeed > 0) {
          wristSpeed = 0;
        }
      }
    }
  }
}