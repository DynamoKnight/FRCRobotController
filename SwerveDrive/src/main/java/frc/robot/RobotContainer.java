// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  /** Max speed (m/s) of robot when at 12 volts. */
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  /** Max rotational speed (rad/s) of the robot. 
   * 1.5π rad/s is 75% of a full rotation (2π). This is for easier control and less motor stress.*/
  private double MaxAngularRate = 1.5 * Math.PI;

  /* Setting up bindings for necessary control of the swerve drive platform */
  // Initializes the controller
  private final CommandXboxController joystick = new CommandXboxController(0);
  // Initiializes the drivetrain subsystem
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  /* Swerve Requests represent how the drivetrain should act. Commands will apply these requests. */
  /** A request that defines how the driving should act.
   *  Field Centric means the robot drives relative to the field,
   *  The joysticks have a 10% deadband,
   *  Open loop control directly sets the output from the input, without feedback. (As opposed to Close loop control)
   */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  /**
   * A request that defines how the braking should act.
   * Swerve Drive Brake means that the modules point in a way that opposes motion.
   */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  /**
   * A request that defines a direction to angle the wheels.
   */
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Configues the swerve requests to commands
  private void configureBindings() {
    // Drivetrain will execute this command periodically
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() -> drive
        // Drive forward with negative Y (forward)
        .withVelocityX(-joystick.getLeftY() * MaxSpeed)
        // Drive left with negative X (left)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
        // Drive counterclockwise with negative X (left)
        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
      ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
      .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // Checks if in simulation
    if (Utils.isSimulation()) {
      // Sets the robot to (0,0) facing left (90 degrees)
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /**
   * Consturctor
   */
  public RobotContainer() {
    configureBindings();
  }

  /**
   * Returns an Auto command
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
