package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

public class LaunchCommand extends Command {

    Timer launchTimer;
    LauncherSubsystem launcher;

    public LaunchCommand(LauncherSubsystem launcher) {
        launchTimer = new Timer();
        this.launcher = launcher;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        launchTimer.reset();
    }

    @Override
    public void execute() {
        launcher.setUpperVoltage(12);
        if (launchTimer.get() > 2) {
            launcher.setLowerVoltage(4);
        }
    }
    
}
