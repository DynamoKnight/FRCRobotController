package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LauncherSubsystem;

public class MiddleAuto extends SequentialCommandGroup{
    /**
     * Creates a sequence of commands for autonomous.
     * @param drivetrain The drivetrain subsystem used to drive the robot.
     * @param launcher The launcher subsystem used to shoot the notes.
     */
    public MiddleAuto(DriveTrain drivetrain, LauncherSubsystem launcher) {
        addCommands(
            // Drives back
            new InstantCommand(() -> drivetrain.moveForward(-0.1), drivetrain),
            new WaitCommand(0.9),
            // Stops briefly for 1 second, calls stop() method
            new InstantCommand(drivetrain::stop, drivetrain),
            new WaitCommand(1),
            // Launches the note to the speaker
            new InstantCommand(() -> launcher.launch(launcher.spkrUpPower, launcher.spkrLowPower)),
            new WaitCommand(3),
            // Drives back to clear the white line
            new InstantCommand(() -> drivetrain.moveForward(-0.1), drivetrain),
            new WaitCommand(1.7),
            // Stop
            new InstantCommand(drivetrain::stop, drivetrain)

            // Turns the robot at 10% power for the 5 seconds
            //new InstantCommand(() -> drivetrain.turn(0.1), drivetrain), new WaitCommand(5),
        );
    }
}
