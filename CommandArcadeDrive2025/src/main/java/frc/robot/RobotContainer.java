// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleAuto;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  // The controller
  public static CommandXboxController controller1 = new CommandXboxController(Constants.Operator.CONTROLLER_PORT_0);
  // For autonomous
  SendableChooser<Command> sc = new SendableChooser<Command>();

  /**
   * The container for the robot.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Adds an auto
    sc.addOption("SimpleAuto", new ExampleAuto(drivetrainSubsystem));
    // The default
    sc.setDefaultOption("SimpleAuto", new ExampleAuto(drivetrainSubsystem));
    // Adds onto SmartDashboard for telemetry
    SmartDashboard.putData("AutoChooser", sc);
  }

  /**
   * Map commands to buttons.
   */
  private void configureButtonBindings() {
    drivetrainSubsystem.setDefaultCommand(
      new DriveCommand(
        drivetrainSubsystem,
        () -> -controller1.getLeftY(),   // move supplier, needs to be inversed
        () -> controller1.getRightX()   // turn supplier
      )
    );
    // Schedules a command when the trigger condition is true
    new Trigger(controller1.leftTrigger()).whileTrue(drivetrainSubsystem.driveFast()).onFalse(drivetrainSubsystem.driveSlow());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // It will run the selected command (there are no commands to select tho)
    return sc.getSelected();
  }

}