package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LauncherSubsystem;

public class LeftAuto extends SequentialCommandGroup{
    /**
     * Creates a sequence of commands for autonomous.
     * @param drivetrain The drivetrain subsystem used to drive the robot.
     * @param launcher The launcher subsystem used to shoot the notes.
     */
    public LeftAuto(DriveTrain drivetrain, LauncherSubsystem launcher) {
        addCommands(
            // Launches the note to the speaker
            new InstantCommand(() -> launcher.launch(launcher.spkrUpPower, launcher.spkrLowPower)),
            new WaitCommand(3),
            // Turns left
            new InstantCommand(() -> drivetrain.turn(-0.1), drivetrain),
            new WaitCommand(0.6),
            // Drives back
            new InstantCommand(() -> drivetrain.moveForward(-0.1), drivetrain),
            new WaitCommand(3),
            new InstantCommand(drivetrain::stop, drivetrain)
        );
    }
}