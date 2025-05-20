// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;


public class DriveTrain extends SubsystemBase {
  
  // The 4 motors of the base
  SparkMax frontLeftMotor = new SparkMax(1, MotorType.kBrushless);
  SparkMax backLeftMotor = new SparkMax(2, MotorType.kBrushless);
  SparkMax frontRightMotor = new SparkMax(3, MotorType.kBrushless);
  SparkMax backRightMotor = new SparkMax(4, MotorType.kBrushless);

  
  // Driver Controller
  CommandXboxController controller = null;
  // Indicates Driver Mode
  double driverMode = 0;
  boolean togglePressed = false;
  // Constants
  double movePower = 0.3;
  double turnPower = 0.3;
  double fastMovePower = 1;
  double fastTurnPower = 0.7;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    // Creates a configuration to apply to motors
    SparkMaxConfig config = new SparkMaxConfig();
    // Sets to brake when robot isn't given power
    config.idleMode(IdleMode.kBrake);
    // Inverts direction
    SparkMaxConfig invertConfig = config;
    invertConfig.inverted(true);
    // Reset mode resets the already saved flash settings to default values, Persist mode saves current settings across cycles
    frontLeftMotor.configure(invertConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    backLeftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    frontRightMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    backRightMotor.configure(invertConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    // TESTING BRANCH
  }

  // Sets the controller
  public void setController(CommandXboxController driveController) {
    controller = driveController;
  }

  /**
   * Example command factory method.
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // Checks if controller is connected
    if (controller != null) {
      // Represents the power from 0-1 of the joysticks axes
      double leftY;
      double rightX;
      // Toggles type of drive if Back Button and Stat button is pressed at same time
      // Makes sure it doens't re-enter clause if held. 
      if (controller.back().getAsBoolean() && controller.start().getAsBoolean() && !togglePressed){
        togglePressed = true;
        if (driverMode == 0){
          driverMode = 1;
          System.out.println("DRIVE-TURN Mode ACTIVE");
        }
        else if (driverMode == 1){
          System.out.println("RACECAR Mode ACTIVE");
          driverMode = 0;
        }
      }
      else{
        togglePressed = false;
      }
      // Drive-Turn movement
      if (driverMode == 0){
        // The input is squared to make smaller values smaller, allowing a gradual increase
        // Math.signum returns the sign of the input, which needs to be preserved
        // It negated because the joysticks are inverted for some reason!
        // Left Y is how forward/backwards which come from the movement of the joystick in the +- Y direction
        rightX = -Math.signum(controller.getLeftY()) * Math.pow(controller.getLeftY(), 2) * fastMovePower;
        // Right X is how much rotation which comes from the joysticks +- X axis
        leftY = -Math.signum(controller.getRightX()) * Math.pow(controller.getRightX(), 2) * fastTurnPower;
      }
      // Rocket League movement (ADRIAN)
      else{
        // Goes forward
        if (controller.getRightTriggerAxis() > 0.1){
          leftY = Math.signum(controller.getRightTriggerAxis()) * Math.pow(controller.getRightTriggerAxis(), 2) * fastMovePower;
        }
        // Goes backwards
        else if (controller.getLeftTriggerAxis() > 0.1){
          leftY = -Math.signum(controller.getLeftTriggerAxis()) * Math.pow(controller.getLeftTriggerAxis(), 2) * fastMovePower;
        }
        // Doesnt move
        else{
          leftY = 0;
        }
        rightX = -Math.signum(controller.getLeftX()) * Math.pow(controller.getLeftX(), 2) * fastTurnPower;
      }

      /*// Sets the deadzone of the joysticks
      if (Math.abs(controller.getLeftY()) < 0.1) {
        leftY = 0;
      }
      if (Math.abs(controller.getRightX()) < 0.1) {
        rightX = 0;
      }*/
      
      // Calculates the vectors
      double rightSide = leftY + rightX;
      double leftSide = leftY - rightX;

      // Finds max power
      double max = Math.max(Math.abs(rightSide), Math.abs(leftSide));
      // Normalizes speed based on the max (making range 0-1)
      if (max > 1) {
        rightSide /= max;
        leftSide /= max;
      }

      // Sets the power of the motors
      // Makes sure it doesn't clash with auto
      if (!DriverStation.isAutonomous()){
        frontLeftMotor.set(leftSide);
        backLeftMotor.set(leftSide);
        frontRightMotor.set(rightSide);
        backRightMotor.set(rightSide);
      }
      
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

   /**
   * Moves the robot forward by setting power to both sides.
   * @param power Positive or negative power for the motor
   */
  public void moveForward(double power) {
    frontLeftMotor.set(power);
    backLeftMotor.set(power);
    frontRightMotor.set(power);
    backRightMotor.set(power);
  }

  /**
   * Turns the robot by spinning the motors in opposite directions.
   * @param power Power to turn (positive for clockwise, negative for counterclockwise)
   */
  public void turn(double power) {
    frontLeftMotor.set(power);
    backLeftMotor.set(power);
    frontRightMotor.set(-power);
    backRightMotor.set(-power);
  }

  /*** Stops all the motors.*/
  public void stop() {
    frontLeftMotor.set(0);
    backLeftMotor.set(0);
    frontRightMotor.set(0);
    backRightMotor.set(0);
  }


}
