package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DumpRollerSubsystem;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.wpilibj.Timer;

public class IntakeCoral extends Command{

    Timer timer = new Timer();

    DumpRollerSubsystem dumpRoller;

    // Buffer to store the last few current readings
    private static final int BUFFER_SIZE = 15;
    private Queue<Double> currentBuffer = new LinkedList<>();

    // Current threshold values
    private static final double HIGH_CURRENT_THRESHOLD = 22;  // Coral is engaged
    private static final int HIGH_CURRENT_COUNT = 4; // Number of readings needed above threshold


    // CONSTRUCTOR
    public IntakeCoral(RobotContainer robotcontainer){
        this.dumpRoller = robotcontainer.dumpRoller;
    }  
    
    @Override
    public void initialize(){
        timer.restart();
        currentBuffer.clear();
        //dumpRoller.coralMotor.set(0.2);
    }

    // Called every 20ms to perform actions of Command
    @Override
    public void execute(){
        dumpRoller.coralMotor.set(0.2);

        // Get current motor draw
        double current = dumpRoller.coralMotor.getOutputCurrent();
        System.out.println("Current: " + current);

        // Maintain buffer size
        if (currentBuffer.size() >= BUFFER_SIZE) {
            currentBuffer.poll(); // Remove oldest value
        }
        currentBuffer.add(current);
    }

    // Called every 20ms to check if command is ended
    @Override
    public boolean isFinished(){
        if (timer.get() > 1) { // Only check after 1 second
            int highCurrentCount = 0;

            // Count number of values above threshold
            for (double val : currentBuffer) {
                if (val > HIGH_CURRENT_THRESHOLD) {
                    highCurrentCount++;
                }
            }

            // Stop only if a majority of recent values are high
            if (highCurrentCount >= HIGH_CURRENT_COUNT) {
                System.out.println("Coral detected! Stopping motor.");
                return true;
            }
        }
        return false;
    }

    // Called once Command ends
    @Override
    public void end(boolean interrupted){
        dumpRoller.coralMotor.set(0);
    }

}
