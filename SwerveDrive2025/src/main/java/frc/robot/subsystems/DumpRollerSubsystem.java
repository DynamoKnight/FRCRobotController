package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class DumpRollerSubsystem extends SubsystemBase{
    
    CommandXboxController controller;
    SparkMax coralMotor = new SparkMax(2, MotorType.kBrushless);
    Timer a_timer = new Timer();
    // Indicates if the launcher is in action
    Boolean isRunning = false;

    double power = 0.5;

    // Initializes the motors and controller
    public DumpRollerSubsystem(CommandXboxController controller) {
        this.controller = controller;
    }


    @Override
    public void periodic() {
        // Outakes on Right trigger 
        if (controller.rightTrigger().getAsBoolean()){
            coralMotor.set(power);
        }
        else{
            coralMotor.set(0);
        }
    }
}
