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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ReefPos;
import frc.robot.commands.AlignReef;
import frc.robot.commands.IntakeCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmWheelSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DumpRollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotContainer {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7; //  kSpeedAt12Volts desired top speed
    public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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
    public final CommandXboxController joystick = new CommandXboxController(0);
    public final CommandXboxController joystick2 = new CommandXboxController(1);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final DumpRollerSubsystem dumpRoller = new DumpRollerSubsystem();
    public final LimelightSubsystem limelight = new LimelightSubsystem(this);
    public final ArmSubsystem arm = new ArmSubsystem();
    public final ArmWheelSubsystem wheel = new ArmWheelSubsystem();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Creates a Named Command, that can be accessed in path planner5
        // Raises Elevator to Level 4
        NamedCommands.registerCommand("Raise L4", elevator.setPositionwithThreshold(4));
        // Raises Elevator to Level 0
        NamedCommands.registerCommand("Raise L0", elevator.setPositionwithThreshold(0));
        // Outtakes Dump Roller on Reef
        NamedCommands.registerCommand("Score", dumpRoller.dropCoral(.2).withTimeout(.5));
        // Stops the Dump Roller
        NamedCommands.registerCommand("Stop Dump Roller", dumpRoller.keepCoral().withTimeout(.01));
        // Aligns to the Left Reef side
        NamedCommands.registerCommand("Align Left", new AlignReef(this, ReefPos.LEFT).withTimeout(1.0));
        // Aligns to the Right Reef side
        NamedCommands.registerCommand("Align Right", new AlignReef(this, ReefPos.RIGHT).withTimeout(1.0));
        // Runs the Dump Roller until coral is detected
        NamedCommands.registerCommand("Intake Coral", new IntakeCoral(this));
        

        // Adds Autos to SmartDashboard
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // DO NOT CHANGE.
        // Open Loop doesn't use feedback, Close Loop uses feedback
        elevator.setDefaultCommand(elevator.setOpenLoop(() -> 0.2));
        dumpRoller.setDefaultCommand(dumpRoller.keepCoral());
        arm.setDefaultCommand(arm.stopArm());
        wheel.setDefaultCommand(wheel.stopWheels());

        // Configures the Bindings
        configureBindings();
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
        // This was causing the issue with overdrive
        //joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //    point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        //));

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

        // Reset the field-centric heading on left bumper press
        joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Aligns to the reef april tag, right side
        joystick.rightBumper().whileTrue(new AlignReef(this, Constants.ReefPos.RIGHT));
        //.onFalse(Commands.startEnd(() -> joystick2.setRumble(RumbleType.kBothRumble, 0.7), () -> joystick2.setRumble(RumbleType.kBothRumble, 0)).withTimeout(0.25));
        // Aligns to the reef april tag, left side
        joystick.leftBumper().whileTrue(new AlignReef(this, Constants.ReefPos.LEFT));
        //.onFalse(Commands.startEnd(() -> joystick2.setRumble(RumbleType.kBothRumble, 0.7), () -> joystick2.setRumble(RumbleType.kBothRumble, 0)).withTimeout(0.25));;

        // Overdrive button for speed
        joystick.b().whileTrue(increaseSpeed()).onFalse(decreaseSpeed()); 

        // Outtakes coral 
        joystick.rightTrigger().onTrue(CoralOuttake());
        

        /////////////////////////////
        // OPERATOR CONTROL
        /////////////////////////////
        joystick2.setRumble(RumbleType.kBothRumble, 0);
        // TOGGLE ELEVATOR POSITIONS
        // Level 0
        joystick2.b().onTrue(elevator.setPosition(0));
        // Level 1
        joystick2.rightTrigger().onTrue(elevator.setPosition(1));
        // Level 2
        joystick2.a().onTrue(elevator.setPosition(2));
        // Level 3
        joystick2.x().onTrue(elevator.setPosition(3));
        // Level 4
        joystick2.y().onTrue(elevator.setPosition(4));
        // ARM CONTROLS
        // Moves Arm Upwards
        joystick2.rightBumper().whileTrue(arm.moveUp());
        // Moves Arm Downwards
        joystick2.leftBumper().whileTrue(arm.moveDown());

        // Spins Wheels Clockwise, Left trigger safety button
        joystick2.povUp().whileTrue(wheel.clockwiseWheels());
        // Spins Wheels Counter-Clockwise, Left trigger safety button
        joystick2.povDown().whileTrue(wheel.counterClockwiseWheels());

        /*// Sticks coral out to make it easier to target
        joystick2.povLeft().onTrue(dumpRoller.PrepareCoral(true));
        // Takes coral back in incase it is falling out
        joystick2.povRight().onTrue(dumpRoller.PrepareCoral(false));*/

        // Sticks coral out when holding Left Dpad
        joystick2.povLeft().whileTrue(dumpRoller.dropCoral(0.15)).onFalse(dumpRoller.keepCoral().withTimeout(0.1));
        // Sticks coral in when holding Right Dpad
        joystick2.povRight().whileTrue(dumpRoller.dropCoral(-0.15)).onFalse(dumpRoller.keepCoral().withTimeout(0.1));

        // Continually runs Dump Roller on Left Trigger until coral detected
        joystick2.leftTrigger().onTrue(new IntakeCoral(this));

    }

    // Outtakes Dump Roller Coral onto Reef
    private Command CoralOuttake(){
        // List of speeds for each elevator level (0-4)
        // Position 0 uses a slower speed (0.1), all other positions use 0.2
        double[] launchSpeeds = {0.1, 0.2, 0.2, 0.2, 0.2};
        
        // Safety check to prevent array index out of bounds - Im a genius
        int posIndex = Math.min(elevator.pos, launchSpeeds.length - 1);
        double speedToUse = launchSpeeds[posIndex];
        
        System.out.println("Elevator position: " + elevator.pos + 
                           ", Rotation value: " + elevator.positions[elevator.pos] + 
                           ", Speed: " + speedToUse);

        // Sequence of Commands
        return Commands.sequence(   
            dumpRoller.dropCoral(speedToUse).withTimeout(0.5), // This needs to be tested, switch speedToUse back to 0.2 if testing fails.
            dumpRoller.keepCoral().withTimeout(0.01),
            // Drops to Level 0 after done
            elevator.setPosition(0)
        );
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public Command increaseSpeed(){
        //run() is now runOnce() because run() keeps executing the speed-setting code in a loop
        return Commands.runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    }

    public Command decreaseSpeed(){
        //run() is now runOnce() because run() keeps executing the speed-setting code in a loop
        return Commands.runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7);
    }

    public void setSpeed(double speed){

    }
    
}
