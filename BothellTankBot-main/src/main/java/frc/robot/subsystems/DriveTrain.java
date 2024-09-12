// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class DriveTrain extends SubsystemBase {
  
  //4 motors of the base
  CANSparkMax frontLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax backLeftMotor = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax frontRightMotor = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax backRightMotor = new CANSparkMax(4, MotorType.kBrushless);

  CommandXboxController controller = null;
  double movePower = 0.3;
  double turnPower = 0.3;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    //reset
    frontRightMotor.restoreFactoryDefaults();
    frontLeftMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();
    backLeftMotor.restoreFactoryDefaults();
    //when idle, robot brakes/stops immediately
    frontRightMotor.setIdleMode(IdleMode.kBrake);
    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    backRightMotor.setIdleMode(IdleMode.kBrake);
    backLeftMotor.setIdleMode(IdleMode.kBrake);
    //inverts front and back
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

  }


  public void setController(CommandXboxController driveController) {
    controller = driveController;
  }

  /**
   * Example command factory method.
   *
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
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    
    //if controller exists
    if (controller != null) {
      //represents power 0 -> 1
      double leftY;
      double rightX;
      //if holding left and right trigger
      if (controller.getLeftTriggerAxis() > 0.5 && controller.getRightTriggerAxis() > 0.5) {
        //regular movement
        turnPower = 0.7;
        leftY = -controller.getLeftY() * movePower;
        rightX = -controller.getRightX() * turnPower;
        //if only holding left trigger, slower forward and back
      } else if (controller.getLeftTriggerAxis() > 0.5) {
        leftY = -controller.getLeftY() * 0.5;
        rightX = -controller.getRightX();
        //if only holding right, slower turning
      } else if (controller.getRightTriggerAxis() > 0.5) {
        leftY = -controller.getLeftY();
        rightX = -controller.getRightX() * 0.5;
        //no trigger pressed, both slower
      } else {
      leftY = -controller.getLeftY() * 0.5;
      rightX = -controller.getRightX() * 0.5;
      }
      //deadzone
      if (Math.abs(leftY) < 0.2) {
        leftY = 0;
      }
      if (Math.abs(rightX) < 0.2) {
        rightX = 0;
      }

      double rightSide = leftY + rightX;
      double leftSide = leftY - rightX;

      //finds max power, normalizes speed based on max (making range 0 -> 1)
      double max = Math.max(Math.abs(rightSide), Math.abs(leftSide));

      if (max > 1) {
        rightSide /= max;
        leftSide /= max;
      }
      // double left = 0.1;
      // double right = 0.1;
      frontRightMotor.set(rightSide);
      frontLeftMotor.set(leftSide);
      backRightMotor.set(rightSide);
      backLeftMotor.set(leftSide);
    }
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
