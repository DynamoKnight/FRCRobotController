// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {

  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(DriveTrain subsystem){
    // Executes the 2 commands in sequence
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  /*** Instantiates the Middle Auto sequence*/
  public static Command middleAuto(DriveTrain drivetrain, LauncherSubsystem launcher) {
    return new MiddleAuto(drivetrain, launcher);
  }
  /*** Instantiates the Left Auto sequence*/
  public static Command leftAuto(DriveTrain drivetrain, LauncherSubsystem launcher) {
    return new LeftAuto(drivetrain, launcher);
  }
  /*** Instantiates the Right Auto sequence*/
  public static Command rightAuto(DriveTrain drivetrain, LauncherSubsystem launcher) {
    return new RightAuto(drivetrain, launcher);
  }
  /*** Instantiates the Right Auto sequence that goes to corner*/
  public static Command rightAutoEsc(DriveTrain drivetrain, LauncherSubsystem launcher) {
    return new RightAutoEsc(drivetrain, launcher);
  }

  // This is a utility class, meanning it's not meant to be instantiated. Its only used for static methods.
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
