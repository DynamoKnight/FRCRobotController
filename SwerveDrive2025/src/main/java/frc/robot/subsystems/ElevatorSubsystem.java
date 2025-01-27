package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
//import com.ctre.phoenix6.motorcontrol.ControlMode;

public class ElevatorSubsystem extends SubsystemBase{
    public TalonFX backElevator = new TalonFX(14, "canivore");
    public TalonFX frontElevator = new TalonFX(15, "canivore");

    CommandXboxController controller;
    Timer a_timer = new Timer();
    // Indicates if the elevator is in action
    Boolean isRunning = false;
    // A list of the number of rotations for each reef level
    private double[] rotations = {2 ,4, 6, 8};
    private int idx = 0;
    // The speed of the motors
    double motorPower = 0.1;

    // Initializes the motors and controller
    public ElevatorSubsystem(CommandXboxController controller) {
        this.controller = controller;
        // Configures the motors
        configureMotor(frontElevator);
        configureMotor(backElevator);
    }

    // Configures settings for motor at start
    private void configureMotor(TalonFX motor) {
        // Resets to factory defaults
        motor.getConfigurator().apply(new TalonFXConfiguration());
        // Ensures the motor power is set to 0
        motor.setControl(new DutyCycleOut(0));
        // Sets the current limit
        var currentConfiguration = new CurrentLimitsConfigs();
        currentConfiguration.StatorCurrentLimit = 80;
        currentConfiguration.StatorCurrentLimitEnable = true;
        // Refreshes and applies the current
        motor.getConfigurator().refresh(currentConfiguration);
        motor.getConfigurator().apply(currentConfiguration);
    }

    @Override
    public void periodic() {
        // Intakes on A button
        if (controller.a().getAsBoolean()) {
            setMotorPower(motorPower);
            // Retrieve and print telemetry values
            System.out.println("Front Elevator power: " + frontElevator.getDutyCycle().getValue());
        } 
        else {
            setMotorPower(0);
        }
    }

    // Sets power of both motors
    public void setMotorPower(double speed){
        frontElevator.setControl(new DutyCycleOut(speed));
        backElevator.setControl(new DutyCycleOut(speed));
    }

    /** Sets the position of both motors.
     * @param rotations is the number of rotations it will hold at.
     */
    public void setPositions(double rotations){
        frontElevator.setPosition(rotations);
        backElevator.setPosition(rotations);
    }

    /** Sets the position of both motors.
     * @param rotations is the number of rotations it will hold at.
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     */
    public void setPositions(double rotations, double timeoutSeconds){
        frontElevator.setPosition(rotations, timeoutSeconds);
        backElevator.setPosition(rotations, timeoutSeconds);
    }

    /** Holds the elevator at reef level 1. */
  public Command setReefLevel1() {
    return this.runOnce(() -> setPositions(2));
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

}