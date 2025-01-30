package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
//import com.ctre.phoenix6.motorcontrol.ControlMode;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorSubsystem extends SubsystemBase{
    public TalonFX backElevator = new TalonFX(14, "canivore");
    public TalonFX frontElevator = new TalonFX(15, "canivore");

    CommandXboxController controller;
    Timer a_timer = new Timer();
    // Indicates if the elevator is in action
    Boolean isRunning = false;
    // A list of the number of rotations for each reef level
    private double holdPosition = 0;
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
        var currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.StatorCurrentLimit = 80;
        currentConfigs.StatorCurrentLimitEnable = true;
        // Refreshes and applies the current
        motor.getConfigurator().refresh(currentConfigs);
        motor.getConfigurator().apply(currentConfigs);
        // Brakes motor when not receiving power
        motor.setNeutralMode(NeutralModeValue.Brake);
        holdPosition = 0;
        motor.setPosition(holdPosition);

        // PID Constants
        // Sets slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
        motor.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void periodic() {
        if (controller.a().getAsBoolean()){
            // Creates a position closed-loop request, voltage output, slot 0 configs
            final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
            // Sets position
            frontElevator.setControl(m_request.withPosition(1));
            backElevator.setControl(m_request.withPosition(1));
        }
        // Moves up on dpad up button
        if (controller.povUp().getAsBoolean()) {
            setMotorPower(motorPower);
            //System.out.println(frontElevator.getDutyCycle().getValue()); // Returns the power
            // Updates the hold position to the current position
            holdPosition = frontElevator.getPosition().getValueAsDouble();
        }
        // Moves down on dpad down button
        else if (controller.povDown().getAsBoolean()) {
            setMotorPower(-motorPower);
            holdPosition = frontElevator.getPosition().getValueAsDouble();
        }
        else {
            setMotorPower(0);
            //setPositions(holdPosition);
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