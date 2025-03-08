package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
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
    private double[] positions = new double[]{0, 10, 15};
    private int posIdx = 0;
    private double armSpeed = 0.2;
    private double spinSpeed = 0.5;

    public double kFF = 0.1;

    SparkMax armMotor = new SparkMax(3, MotorType.kBrushless);
    SparkMax spinMotor = new SparkMax(4, MotorType.kBrushless);
    private final RelativeEncoder encoder;

    public ArmSubsystem() {
        // Get and configure the encoder
        encoder = armMotor.getEncoder();
        //encoder.setPosition(0);

        // Create and apply configuration for arm motor. Voltage compensation helps
        // the arm behave the same as the battery
        // voltage dips. The current limit helps prevent breaker trips or burning out
        // the motor in the event the arm stalls.
        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.voltageCompensation(10);
        //armConfig.smartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);
        armConfig.idleMode(IdleMode.kBrake);
        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    public Command moveUp() {
        return Commands.run(() -> armMotor.set(armSpeed));
    }
    
    // Command to move the arm down
    public Command moveDown() {
        return Commands.run(() -> armMotor.set(-armSpeed));
    }

    // Command to move the arm to a set position
    public Command setArmDown() {
        pid.setSetpoint(encoder.getPosition() - positions[posIdx]);
        return setArm();
    }

    public Command setArmUp() {
        if (posIdx >= positions.length - 1){
            return null;
        }
        else{
            posIdx += 1;
            pid.setSetpoint(encoder.getPosition() + positions[posIdx]);
            return setArm();
        }
        
    }

    public Command setArm() {
        return Commands.run(() -> {
            double velocity = pid.calculate(encoder.getPosition());
            velocity = MathUtil.clamp(velocity, -armSpeed, armSpeed);
            armMotor.set(velocity);
        });
    }

    public Command spinWheels(){
        return Commands.run(() -> spinMotor.set(spinSpeed));
    }

    public Command stopWheels(){
        return Commands.run(() -> spinMotor.set(0));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Rotations", encoder.getPosition());
    }
}