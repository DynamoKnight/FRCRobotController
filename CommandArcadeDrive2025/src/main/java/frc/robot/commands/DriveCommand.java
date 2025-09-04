// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;

/** Allows the robot to drive teleoperationally */
public class DriveCommand extends Command {
  private final DrivetrainSubsystem drivetrainSubsystem;
  // A double supplier is anything that retuns a double
  private final DoubleSupplier moveSupplier;
  private final DoubleSupplier turnSupplier;

  public DriveCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.moveSupplier = moveSupplier;
    this.turnSupplier = turnSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting controller drive command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculates power needed for left and right side power
    // Left stick Y = forward/backward
    double move = Math.signum(moveSupplier.getAsDouble()) * Math.pow(moveSupplier.getAsDouble(), 2) * drivetrainSubsystem.movePower;
    // Right stick X = clockwise/counter-clockwise
    double turn = Math.signum(turnSupplier.getAsDouble()) * Math.pow(turnSupplier.getAsDouble(), 2) * drivetrainSubsystem.turnPower;
    // How much power is needed for each side is dependent on move and turn
    double leftPower = move + turn;
    double rightPower = move - turn;

    // Normalizes speed to a range of 0-1
    double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
    if (max > 1.0) {
      leftPower /= max;
      rightPower /= max;
    }

    drivetrainSubsystem.tankDrive(leftPower, rightPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}