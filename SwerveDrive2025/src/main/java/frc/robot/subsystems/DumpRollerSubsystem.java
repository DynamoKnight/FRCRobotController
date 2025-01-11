package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class DumpRollerSubsystem extends SubsystemBase{
    
    CommandXboxController controller;
    SparkMax motorController = new SparkMax(1, MotorType.kBrushed);
    Timer a_timer = new Timer();
    // Indicates if the launcher is in action
    Boolean isRunning = false;

    double intakePower = 0.5;

    // Initializes the motors and controller
    public DumpRollerSubsystem(CommandXboxController controller) {
        this.controller = controller;
    }

    @Override
    public void periodic() {
        // Intakes on A button
        if (controller.a().getAsBoolean()) {
            motorController.set(intakePower);
        }
        else{
            motorController.set(0);
        }
    }
}
