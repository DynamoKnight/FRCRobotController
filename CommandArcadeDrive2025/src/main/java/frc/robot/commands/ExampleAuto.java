package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ExampleAuto extends SequentialCommandGroup{
    /**
     * Creates a sequence of commands for autonomous.
     * @param drivetrain The drivetrain subsystem used to drive the robot.
     */
    public ExampleAuto(DrivetrainSubsystem drivetrain) {
        addCommands(
            // Drives forward
            new InstantCommand(() -> drivetrain.moveTank(0.1), drivetrain),
            new WaitCommand(3),
             // Turns left
            new InstantCommand(() -> drivetrain.turnTank(-0.1), drivetrain),
            new WaitCommand(0.6),
            new InstantCommand(drivetrain::stop, drivetrain)
        );
    }
}