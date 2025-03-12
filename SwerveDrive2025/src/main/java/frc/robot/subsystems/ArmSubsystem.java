package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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
    private double armHold = .1;

    SparkMax armMotor = new SparkMax(3, MotorType.kBrushless);
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
    
    // Command to stop the arm and hold at position
    public Command stopArm() {
        return Commands.run(() -> armMotor.stopMotor());
    }

    // Get the current position of the arm
    public double getArmPosition() {
        return encoder.getPosition();
    }
    
    // Command to move the arm up continuously
    public Command moveUp() {
        return Commands.run(() -> armMotor.set(armSpeed),this);
    }
    
    // Command to move the arm down continuously
    public Command moveDown() {
        return Commands.run(() -> armMotor.set(-armSpeed),this);
    }

    // Command to move the arm to the bottom position
    public Command setArmDown() {
        posIdx = 0;
        pid.setSetpoint(encoder.getPosition() - positions[posIdx]);
        return setArm();
    }

    // Command to move the arm up one level
    public Command setArmUp() {
        // Checks if next position is possible
        if (posIdx >= positions.length - 1){
            return null;
        }
        else{
            posIdx += 1;
            pid.setSetpoint(encoder.getPosition() + positions[posIdx]);
            return setArm();
        }
        
    }

    // Moves the arm based on the setpoint
    public Command setArm() {
        return Commands.run(() -> {
            double velocity = pid.calculate(encoder.getPosition());
            velocity = MathUtil.clamp(velocity, -armSpeed, armSpeed);
            armMotor.set(velocity);
        });
    }

    

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Rotations", encoder.getPosition());
    }
}