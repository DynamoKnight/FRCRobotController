package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

        /* ----- PIDs ----- */
    private PIDController pid = new PIDController(1, 0, 0);
    private double[] positions = new double[]{0, 10, 15};
    private int posIdx = 0;
    private double armSpeed = 0.2;

    private final TalonFX armMotor = new TalonFX(16, "rio");

    public ArmSubsystem() {
       TalonFXConfiguration config = new TalonFXConfiguration();
            // Brakes when there is no output
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            // Applies configuration in 1 second
            armMotor.getConfigurator().apply(config, 1);
    }
    
    // Command to stop the arm and hold at position
    public Command stopArm() {
        return Commands.run(() -> armMotor.set(0),this);
    }
    
    // Command to move the arm up continuously
    public Command moveUp() {
        return Commands.run(() -> armMotor.set(armSpeed),this);
    }
    
    // Command to move the arm down continuously
    public Command moveDown() {
        return Commands.run(() -> armMotor.set(-armSpeed),this);
    }    

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Rotations", armMotor.getPosition().getValueAsDouble());
    }
}