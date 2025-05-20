package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class DumpRollerSubsystem extends SubsystemBase{
    
    // Initializes the motor
    public SparkMax coralMotor = new SparkMax(2, MotorType.kBrushless);
    private DigitalInput coralSensor;
    public Timer a_timer = new Timer();
    // Indicates if the launcher is in action
    public Boolean isRunning = false;
    // Rotation power
    public double power = 1.0;

    // The current threshold to stop the motor
    public final double MAX_CURRENT = 30;
    // Indicates if the coral is being held
    public boolean isHolding = false;

    // Initializes the motors and controller
    public DumpRollerSubsystem() {
        coralSensor = new DigitalInput(9); // On port 9 of the roboRIO
    }

    // Outtakes the coral
    public Command dropCoral(double voltage){
        return Commands.run(() -> coralMotor.set(voltage * power), this);
    }
    
    // Holds the motor
    public Command keepCoral(){
        return Commands.run(() -> coralMotor.set(0), this);
    }

    // Returns the sensor input, If a coral was found
    public boolean getSensorInput() {
        return coralSensor.get();
        }

    // Controls the position of the coral
    public Command PrepareCoral(boolean out){
        // Sticks the coral out
        if (out){
            return Commands.sequence(
                dropCoral(0.15).withTimeout(0.25),
                keepCoral().withTimeout(0.1)
            );
        }
        // Pushes the coral back inside
        else{
            return Commands.sequence(
                dropCoral(-0.2).withTimeout(0.25),
                keepCoral().withTimeout(0.1)
            );
        }
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Coral Motor Current", coralMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Intake Sensor", coralSensor.get());
    }
}
