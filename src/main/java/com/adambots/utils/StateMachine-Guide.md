# State Machine Implementation Guide for FRC
## Understanding State Machines

### What is a State Machine?
A state machine is a model that describes how a system behaves and transitions between different states. Think of it like a flowchart where:
- Each box represents a "state" (a specific condition or mode)
- Each arrow represents a "transition" (how we move between states)
- The system can only be in one state at a time

### Key Concepts

1. **States**: 
   - Represent different modes or conditions of your system
   - Have specific entry/exit conditions
   - Can have associated behaviors
   - Example: An arm might have states like "Ground", "Mid", and "High"

2. **Transitions**:
   - Define how the system moves between states
   - Can have conditions that must be met
   - Can trigger actions when executed
   - Example: Moving from "Ground" to "Mid" position

3. **Triggers**:
   - Conditions that determine when a state is active
   - Can be based on sensor readings, positions, or other criteria
   - Example: An arm is in the "Ground" state when its encoder reads near zero

## Implementation in FRC

### The StateMachine Class
Our StateMachine implementation provides:
- Automatic state management based on triggers
- Type-safe transitions between states
- Built-in validation and error checking
- Debug logging capabilities

### Basic Usage Example
Here's a simple example of how to use the StateMachine:

```java
public class ExampleSubsystem extends SubsystemBase {
    private final StateMachine<MyContext> stateMachine;
    private final StateMachine<MyContext>.State groundState;
    private final StateMachine<MyContext>.State midState;
    
    public ExampleSubsystem() {
        MyContext context = new MyContext();
        stateMachine = new StateMachine<>(context);
        
        // Create states
        groundState = stateMachine.addState("Ground", 
            () -> isAtPosition(0.0));
        midState = stateMachine.addState("Mid", 
            () -> isAtPosition(45.0));
            
        // Add transitions
        groundState.addTransition(midState, 
            ctx -> moveToPosition(45.0));
    }
}
```

## Detailed Implementation: Arm Subsystem Example

### Step 1: Define Your Context
First, create a class to hold your subsystem's state:

```java
public class ArmContext {
    public double currentPosition = 0.0;
    public double targetPosition = 0.0;
    public double motorSpeed = 0.0;
}
```

### Step 2: Create the Subsystem
Here's a complete example of an arm subsystem using the state machine:

```java
public class ArmSubsystem extends SubsystemBase {
    // Hardware components
    private final MotorController armMotor;
    private final DutyCycleEncoder armEncoder;
    
    // State Machine components
    private final ArmContext context;
    private final StateMachine<ArmContext> stateMachine;
    private final StateMachine<ArmContext>.State groundState;
    private final StateMachine<ArmContext>.State midState;
    private final StateMachine<ArmContext>.State highState;
    
    // Constants
    private static final double GROUND_POS = 0.0;
    private static final double MID_POS = 45.0;
    private static final double HIGH_POS = 90.0;
    private static final double TOLERANCE = 2.0;
    
    public ArmSubsystem() {
        // Initialize hardware
        armMotor = new PWMSparkMax(0);
        armEncoder = new DutyCycleEncoder(0);
        
        // Initialize state machine
        context = new ArmContext();
        stateMachine = new StateMachine<>(context);
        
        // Create states with trigger conditions
        groundState = stateMachine.addState("Ground", () ->
            isAtPosition(GROUND_POS) || 
            (context.targetPosition == GROUND_POS && 
             context.motorSpeed < 0));
             
        midState = stateMachine.addState("Mid", () ->
            isAtPosition(MID_POS) || 
            (context.targetPosition == MID_POS && 
             isMovingTowards(MID_POS)));
             
        highState = stateMachine.addState("High", () ->
            isAtPosition(HIGH_POS) || 
            (context.targetPosition == HIGH_POS && 
             context.motorSpeed > 0));
        
        // Define transitions
        groundState.addTransition(midState, ctx -> {
            ctx.targetPosition = MID_POS;
            ctx.motorSpeed = 0.5;
        });
        
        midState.addTransition(highState, ctx -> {
            ctx.targetPosition = HIGH_POS;
            ctx.motorSpeed = 0.5;
        });
        
        midState.addTransition(groundState, ctx -> {
            ctx.targetPosition = GROUND_POS;
            ctx.motorSpeed = -0.5;
        });
        
        highState.addTransition(midState, ctx -> {
            ctx.targetPosition = MID_POS;
            ctx.motorSpeed = -0.5;
        });
    }
    
    private boolean isAtPosition(double position) {
        return Math.abs(armEncoder.getDistance() - position) < TOLERANCE;
    }
    
    private boolean isMovingTowards(double position) {
        double current = armEncoder.getDistance();
        return (current < position && context.motorSpeed > 0) ||
               (current > position && context.motorSpeed < 0);
    }
    
    @Override
    public void periodic() {
        // Update context
        context.currentPosition = armEncoder.getDistance();
        
        // Update state machine
        stateMachine.periodic();
        
        // Apply motor output
        armMotor.set(context.motorSpeed);
        
        // Update SmartDashboard
        SmartDashboard.putString("Current State", 
            stateMachine.getCurrentState().getName());
        SmartDashboard.putNumber("Arm Position", 
            context.currentPosition);
    }
    
    // Public methods for commanding the arm
    public void moveToGround() {
        stateMachine.requestTransition(groundState);
    }
    
    public void moveToMid() {
        stateMachine.requestTransition(midState);
    }
    
    public void moveToHigh() {
        stateMachine.requestTransition(highState);
    }
}
```

### Step 3: Using the Subsystem
In your RobotContainer:

```java
public class RobotContainer {
    private final ArmSubsystem m_arm = new ArmSubsystem();
    
    public RobotContainer() {
        configureButtonBindings();
    }
    
    private void configureButtonBindings() {
        Buttons.XboxAButton
            .onTrue(runOnce(() -> m_arm.moveToGround()));
            
        Buttons.XboxBButton
            .onTrue(runOnce(() -> m_arm.moveToMid()));
            
        Buttons.JoystickButton1
            .onTrue(runOnce(() -> m_arm.moveToHigh()));
    }
}
```

## Best Practices

1. **State Design**:
   - Keep states simple and clearly defined
   - Each state should have a clear purpose
   - Avoid having too many states

2. **Transitions**:
   - Make transitions explicit and intentional
   - Include proper error checking
   - Consider what happens if a transition fails

3. **Trigger Conditions**:
   - Include both position and movement in triggers
   - Add appropriate tolerances
   - Consider edge cases

4. **Safety**:
   - Add limit switch checks
   - Include maximum speed limits
   - Consider adding emergency stop states

5. **Testing**:
   - Test all state transitions
   - Verify trigger conditions
   - Test edge cases and error conditions

## Troubleshooting

Common issues and solutions:

1. **State Not Changing**:
   - Check trigger conditions
   - Verify transition is defined
   - Look at debug logs

2. **Unexpected Transitions**:
   - Check for overlapping trigger conditions
   - Verify state priorities
   - Add more specific conditions

3. **Motor Not Moving**:
   - Check context updates
   - Verify motor assignments
   - Check speed calculations

## Conclusion
State machines provide a robust way to manage complex subsystem behavior. By breaking down your subsystem into clear states and transitions, you can create more reliable and maintainable code. Remember to:
- Keep states and transitions clear and simple
- Test thoroughly
- Use the debug features when needed
- Consider safety at all times

For more help, review the StateMachine code documentation or ask your mentors for guidance.