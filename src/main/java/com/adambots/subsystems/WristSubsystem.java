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
    L4(new WristProperties(ElevatorConstants.kWristL4Position, "Level 4 Angle")),
    HighAlgae(new WristProperties(ElevatorConstants.kWristHighAlgaePosition, "High Algae Angle")),
    LowAlgae(new WristProperties(ElevatorConstants.kWristLowAlgaePosition, "Low Algae Angle")),
    PROCESSOR(new WristProperties(ElevatorConstants.kWristProcessorPosition, "Processor Angle")),
    GROUND_INTAKE(new WristProperties(ElevatorConstants.kWristGroundIntakePosition, "Ground Intake Angle")),
    HANG(new WristProperties(ElevatorConstants.kWristHangPosition, "Hang Angle"));


    public final WristProperties properties;

    WristState(WristProperties properties) {
      this.properties = properties;
    }
  }

  // Hardware
  private final BaseAbsoluteEncoder wristEncoder;
  private final PIDController wristPID;
  private final BaseMotor wristMotor;
  public static boolean isManual = true;
  public static double goalWristAngle;

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
    wristPID.enableContinuousInput(0.0, 360.0);


    configureMotors();

    // Initialize wrist state machine without position control
    wristStateMachine = new StateMachine<>(
        WristState.STOWED,
        WristState.STOWED.properties,
        message -> SmartDashboard.putString("Wrist/Status", message),
        false // Not using position control
    );

    goalWristAngle = 40;
  }

  // public static void setWristAngle()_{
  //   goalWristAngle = wristEncoder.getAbsolutePositionDegrees();
  // }

  private void configureMotors() {
    // Configure wrist motor
    wristMotor.setBrakeMode(true);
    wristMotor.setInverted(true);
  }

  private void setWristOutput(WristProperties properties) {
    setWristPosition(properties.angleDegrees());
    // if (wristPID.atSetpoint()) {
    //   wristSpeed = 0;
    // }
  }

  private boolean isWristAtTarget() {
    return wristPID.atSetpoint();
  }

  public void setWristSpeed(double speed) {
    wristSpeed = speed;
  }

  public void setWristPosition(double angleDegrees) {
    wristSpeed = wristPID.calculate(wristEncoder.getAbsolutePositionDegrees(),
        angleDegrees);
  }

  public void moveWristDown() {
    isManual = true;
    // setWristPosition(wristEncoder.getAbsolutePositionDegrees() - ElevatorConstants.kWristPositionIncrement);
    goalWristAngle = wristEncoder.getAbsolutePositionDegrees() - ElevatorConstants.kWristPositionIncrement;
}
public void moveWristUp() {
    isManual = true;
    // setWristPosition(wristEncoder.getAbsolutePositionDegrees() + ElevatorConstants.kWristPositionIncrement);
    goalWristAngle = wristEncoder.getAbsolutePositionDegrees() + ElevatorConstants.kWristPositionIncrement;
}

  public void holdWristPosition() {
    setWristPosition(wristEncoder.getAbsolutePositionDegrees());
    goalWristAngle = wristEncoder.getAbsolutePositionDegrees();
  }

  @Override
  public void periodic() {

    
    // Update wrist state machine
    wristStateMachine.periodic();
    
    if (!isManual){
      WristState currentState = wristStateMachine.getCurrentState();
      setWristPosition(currentState.properties.angleDegrees());
    } else {
      setWristPosition(goalWristAngle);
    }
    checkFailSafes();
    wristMotor.set(wristSpeed);

    // Get current position
    double currentAngle = wristEncoder.getAbsolutePositionDegrees();

    // Update dashboard
    SmartDashboard.putNumber("Wrist/CurrentAngle", currentAngle);
    SmartDashboard.putNumber("Wrist/Speed", wristSpeed);
    SmartDashboard.putNumber("Wrist/TargetAngle",
        wristStateMachine.getTargetProperties().angleDegrees());
    SmartDashboard.putString("Wrist/State",
        wristStateMachine.getCurrentState().toString());
    SmartDashboard.putBoolean("Wrist/AtTarget", isWristAtTarget());
  }

  // Public methods for commanding the wrist
  public void moveWristToState(WristState state) {
    isManual = false;
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

    double elevatorCurrentPosition = RobotMap.elevatorMotor.getPosition();
    
    // if (wristSpeed > 0 && wristEncoder.getAbsolutePositionDegrees() >= ElevatorConstants.kWristMaxAngle && wristEncoder.getAbsolutePositionDegrees() <= 350) {
    //   // wristSpeed = 0;
    //   holdWristPosition();
    // }
    // if (wristSpeed < 0 && wristEncoder.getAbsolutePositionDegrees() >= ElevatorConstants.kWristMinAngle) {
    //   // wristSpeed = 0;
    //   holdWristPosition();
    // }
    // if (elevatorCurrentPosition >= ElevatorConstants.kElevatorDangerZoneStart
    //     && elevatorCurrentPosition <= ElevatorConstants.kElevatorDangerZoneEnd) {

    //   if (wristEncoder.getAbsolutePositionDegrees() >= ElevatorConstants.kWristDangerZoneAngle) {

    //     if (wristSpeed > 0) {
    //       // wristSpeed = 0;
    //       holdWristPosition();
    //     }
    //   }
    // }
  }
}