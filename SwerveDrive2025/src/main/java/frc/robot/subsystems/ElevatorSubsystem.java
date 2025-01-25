package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.hardware.*;

public class ElevatorSubsystem extends SubsystemBase{
    public TalonFX frontElevator = new TalonFX(14);
    public TalonFX backElevator = new TalonFX(15);

    CommandXboxController controller;
    Timer a_timer = new Timer();
    // Indicates if the elevator is in action
    Boolean isRunning = false;

    double motorPower = 0.5;

    // Initializes the motors and controller
    public ElevatorSubsystem(CommandXboxController controller) {
        this.controller = controller;
    }

    @Override
    public void periodic() {
        // Intakes on A button
        if (controller.a().getAsBoolean()) {
            frontElevator.set(motorPower);
        }
        else{
            frontElevator.set(0);
        }
    }
}
