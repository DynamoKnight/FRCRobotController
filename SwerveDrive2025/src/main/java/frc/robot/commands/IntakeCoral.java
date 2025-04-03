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
    // Indicated if using sensor
    boolean usingSensor = false;
    // Indicates if coral was detected
    boolean coralDetected = false;
    // How much seconds to wait before ending command
    double delayTime = 0.0;

    // Buffer to store the last few current readings
    private static final int BUFFER_SIZE = 15;
    private Queue<Double> currentBuffer = new LinkedList<>();

    // Current threshold value indicating if coral was intaked
    private static final double HIGH_CURRENT_THRESHOLD = 35;
    // Number of readings needed above threshold to confirm coral was intaked
    private static final int HIGH_CURRENT_COUNT = 4;


    // CONSTRUCTOR
    public IntakeCoral(RobotContainer robotcontainer){          
        this.dumpRoller = robotcontainer.dumpRoller;
    }  
    
    @Override
    public void initialize(){
        timer.restart();
        currentBuffer.clear();
        coralDetected = false;
        
    }

    // Called every 20ms to perform actions of Command
    @Override
    public void execute(){
        // Constantly runs the intake
        dumpRoller.coralMotor.set(0.2);

        // Gets current motor draw
        double current = dumpRoller.coralMotor.getOutputCurrent();
        //System.out.println("Current: " + current);

        // Maintains buffer size by only tracking the last few values
        if (currentBuffer.size() >= BUFFER_SIZE) {
            // Removes oldest value
            currentBuffer.poll(); 
        }
        // Tracks newest value
        currentBuffer.add(current);
    }

    // Called every 20ms to check if command is ended
    @Override
    public boolean isFinished(){
        // Uses Sensor to detect Coral
        if (usingSensor){
            // If coral is first detected
            if (dumpRoller.getSensorInput() && !coralDetected){
                // Starts a delay timer
                timer.restart();
                coralDetected = true;
                System.out.println("Coral detected! Stopping motor.");
            }
            // If coral was detected and delay timer has elapsed
            if (coralDetected && timer.hasElapsed(delayTime)){
                return true;
            }
            
        }
        // Uses Current output to detect Coral
        else{
            // Only checks after 1 second (It spikes when starting)
            if (timer.get() > 1) {
                // The count of high currents detected
                int highCurrentCount = 0;
                // Counts number of values above threshold
                for (double val : currentBuffer) {
                    if (val > HIGH_CURRENT_THRESHOLD) {
                        highCurrentCount++;
                    }
                }
                // If there was a lot of high current values, then a coral was detected
                if (highCurrentCount >= HIGH_CURRENT_COUNT) {
                    System.out.println("Coral detected! Stopping motor.");
                    return true;
                }
            }
        }
        return false;
    }

    // Called once Command ends
    @Override
    public void end(boolean interrupted){
        // Stops the Dump Roller
        dumpRoller.coralMotor.set(0);
    }

}
