package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.CANBus;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

        /* ----- PIDs ----- */
    private PIDController pid = new PIDController(1, 0, 0);
    private double[] positions = new double[]{0, 1};
    private double speed = 0.2;

    SparkMax armMotor = new SparkMax(3, MotorType.kBrushless);
    SparkMax spinMotor = new SparkMax(4, MotorType.kBrushless);
    private final RelativeEncoder encoder;

    public ArmSubsystem() {
        // Get and configure the encoder
        encoder = armMotor.getEncoder();
        // encoder.setPosition(0);
    }
    
    // Stop the arm
    public Command stopArm() {
        return Commands.run(() -> armMotor.stopMotor());
    }

    // Get the current position of the arm
    public double getArmPosition() {
        return encoder.getPosition();
    }
    
    // Command to move the arm up
    public Command moveArmUpCommand() {
        return Commands.run(() -> armMotor.set(speed));
    }
    
    // Command to move the arm down
    public Command moveArmDownCommand() {
        return Commands.run(() -> armMotor.set(-speed));
    }

    // Command to move the arm to a set position
    public Command setArm(int pos) {
        pid.setSetpoint(positions[pos]);
        return Commands.run(() -> {
            double velocity = pid.calculate(encoder.getPosition());
            velocity = MathUtil.clamp(velocity, -speed, speed);
            armMotor.set(velocity);
        });
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Rottaions", encoder.getPosition());
    }
}