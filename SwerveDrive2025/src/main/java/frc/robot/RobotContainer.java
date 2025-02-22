// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ReefPos;
import frc.robot.commands.AlignReef;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DumpRollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Joysticks
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem(joystick2);
    public final DumpRollerSubsystem dumpRoller = new DumpRollerSubsystem(joystick2);
    public final LimelightSubsystem limelight = new LimelightSubsystem(this);

     // Represents a list of the number of rotations to get to each level
     Double[] positions = {0.5, 10.5, 21.0, 39.0};
     int pos = 0;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Creates a Named Command, that can be accessed in path planner5
        NamedCommands.registerCommand("Raise L4", elevator.setCloseLoop(() -> 39).until(() -> Math.abs(39 - elevator.getPosition()) < 0.5));
        NamedCommands.registerCommand("Score L4", dumpRoller.dropCoral(0.2).withTimeout(0.5));
        NamedCommands.registerCommand("Lower", elevator.setCloseLoop(() -> 0.5).until(() -> Math.abs(0.5 - elevator.getPosition()) < 0.5));
        NamedCommands.registerCommand("Align Left", new AlignReef(this, ReefPos.LEFT).withTimeout(3));
        NamedCommands.registerCommand("Align Right", new AlignReef(this, ReefPos.RIGHT).withTimeout(3));

        // Adds an auto
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // DO NOT CHANGE.
        // Open Loop doesn't use feedback, Close Loop uses feedback
        elevator.setDefaultCommand(elevator.setOpenLoop(() -> 0.2));
        dumpRoller.setDefaultCommand(dumpRoller.keepCoral());

        // Configures the Bindings
        configureBindings();
    }

    // Sets different powers for different levels
    private Command ToggleCoralOuttake(){
        // Level 1 outake
        if (pos == 0){
            return dumpRoller.dropCoral(.2).withTimeout(0.5);
        }
        // Levels 2, 3, 4 outtake
        else{
            // Drops to Level 1 after done
            return dumpRoller.dropCoral(.3).withTimeout(.5);
        }
         
    }

    // Moves the elevator to the position based on the list
    private Command setPosition(int pos){
        this.pos = pos;
        return elevator.setCloseLoop(() -> positions[pos]);
    }

    // Configures the bindings
    private void configureBindings() {
        /////////////////////////////
        // DRIVER CONTROL
        /////////////////////////////
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        /////////////////////////////
        // OPERATOR CONTROL
        /////////////////////////////
        // Toggle Elevator Positions
        // Level 1
        joystick2.b().onTrue(setPosition(0));
        // Level 2
        joystick2.a().onTrue(setPosition(1));
        // Level 3
        joystick2.x().onTrue(setPosition(2));
        // Level 4
        joystick2.y().onTrue(setPosition(3));
        // Outtakes coral
        joystick2.rightTrigger().whileTrue(ToggleCoralOuttake());

        // Aligns to the reef april tag, right side
        joystick.rightTrigger().whileTrue(new AlignReef(this, Constants.ReefPos.RIGHT));
        // Aligns to the reef april tag, left side
        joystick.leftTrigger().whileTrue(new AlignReef(this, Constants.ReefPos.LEFT));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
    
}
