package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AlignReef extends Command{
    
    private LimelightSubsystem limelight;
    Timer timer = new Timer();


    public AlignReef(){

    }

    @Override
    public void initialize(){
        
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