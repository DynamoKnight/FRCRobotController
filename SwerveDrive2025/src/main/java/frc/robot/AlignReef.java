package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

public class AlignReef extends Command{

    // Subsystems
    private LimelightSubsystem limelight;
    private CommandSwerveDrivetrain drive;

    Timer timer = new Timer();

    private PIDController pidX = new PIDController(3, 0, 0);


    public AlignReef(CommandSwerveDrivetrain drive, LimelightSubsystem limelight){
        this.drive = drive;
        this.limelight = limelight;
    }

    @Override
    public void initialize(){
        Pose2d currentPose = drive.getState().Pose;
        int tid = limelight.getTid();
        //pidX.setSetpoint(1);
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return timer.get() > 2;
    }

    @Override
    public void end(boolean interrupted){

    }

}