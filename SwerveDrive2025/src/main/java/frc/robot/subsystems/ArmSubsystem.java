package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax armMotor = new CANSparkMax(3, MotorType.kBrushless);
    private final RelativeEncoder encoder;

    public ArmSubsystem() {
        // Reset motor controller to defaults
        armMotor.restoreFactoryDefaults();
        
        // Get and configure the encoder
        encoder = armMotor.getEncoder();
        encoder.setPosition(0);
        
        // Set current limit to protect motor
        armMotor.setSmartCurrentLimit(30);
        
        // Use brake mode to hold position when stopped
        armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    // public void moveArm(double speed) {
    //     // Limit to safe speed (Set to 70% max)
    //     double safeSpeed = Math.max(-0.7, Math.min(speed, 0.7));
        
    //     // Apply the speed to the motor
    //     armMotor.set(safeSpeed);
    // }
    
    // Stop the arm
    public void stopArm() {
        armMotor.set(0);
    }

    // Get the current position of the arm
    public double getArmPosition() {
        return encoder.getPosition();
    }
    
    // Command to move the arm up
    public Command moveArmUpCommand() {
        return Commands.run(() -> moveArm(0.5), this);
    }
    
    // Command to move the arm down
    public Command moveArmDownCommand() {
        return Commands.run(() -> moveArm(-0.5), this);
    }
    
    // // Command to stop the arm
    // public Command stopArmCommand() {
    //     return Commands.runOnce(() -> stopArm(), this);
    // }
    
    @Override
    public void periodic(){

    }
}