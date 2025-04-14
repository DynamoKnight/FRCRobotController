package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmWheelSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class AlgaeCollection extends Command {
    ArmSubsystem algaeCollectorArm;
    Timer timer = new Timer();
    ArmWheelSubsystem armWheelSubsystem;
    boolean hasAlgae;

    public AlgaeCollection(ArmSubsystem arm, ArmWheelSubsystem armWheelSubsystem) {
        this.algaeCollectorArm = arm;
        this.armWheelSubsystem = armWheelSubsystem;
    }

    @Override
    public void initialize() {
        hasAlgae = false;
        timer.restart();
        System.out.println("Setting algae collector arm to -0.44");
        algaeCollectorArm.setSetpoint(-0.44);
        armWheelSubsystem.counterClockwiseWheels().schedule();
    }

    @Override
    public void execute() {
        if (timer.get() > 0.25 && Math.abs(armWheelSubsystem.getVelocity()) < 200) {
            hasAlgae = true;
        }
    }

    @Override
    public boolean isFinished() {
        return hasAlgae;
    }

    @Override
    public void end(boolean interrupted) {
        armWheelSubsystem.stopWheels().schedule();
    }

}
