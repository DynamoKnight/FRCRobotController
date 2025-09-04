// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  
  // The 4 motors of the base
  SparkMax frontLeftMotor = new SparkMax(Constants.DrivetrainConstants.frontLeftID, MotorType.kBrushless);
  SparkMax backLeftMotor = new SparkMax(Constants.DrivetrainConstants.backLeftID, MotorType.kBrushless);
  SparkMax frontRightMotor = new SparkMax(Constants.DrivetrainConstants.frontRightID, MotorType.kBrushless);
  SparkMax backRightMotor = new SparkMax(Constants.DrivetrainConstants.backRightID, MotorType.kBrushless);
  // The encoders of the main motors
  RelativeEncoder leftEncoder = frontLeftMotor.getEncoder();
  RelativeEncoder righEncoder = frontRightMotor.getEncoder();
  // Power for movement and turn
  public double movePower = Constants.DrivetrainConstants.slowMovePower;
  public double turnPower = Constants.DrivetrainConstants.slowTurnPower;

  public DrivetrainSubsystem() {
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
  }

  /** Tank drive with left and right side power. */
  public void tankDrive(double left, double right) {
    frontLeftMotor.set(left);
    backLeftMotor.set(left);
    frontRightMotor.set(right);
    backRightMotor.set(right);
  }

  /** Stops all motors. */
  public void stop() {
    tankDrive(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Sets the drive speed fast
  public Command driveFast(){
    return runOnce(
      () -> {
        movePower = Constants.DrivetrainConstants.fastMovePower;
        turnPower = Constants.DrivetrainConstants.fastTurnPower;
    });
  }

  // Sets the drive speed slow (regular)
  public Command driveSlow(){
    return runOnce(
      () -> {
        movePower = Constants.DrivetrainConstants.slowMovePower;
        turnPower = Constants.DrivetrainConstants.slowTurnPower;
    });
  }

}
