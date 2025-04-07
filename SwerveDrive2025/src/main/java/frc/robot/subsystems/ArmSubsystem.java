package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    // The number of rotations to get to each position (needs tuning)
    private double[] positions = new double[]{0, 10, 15};
    // The current position it is at
    private int pos = 0;
    private double armSpeed = 0.2;
    // Initializes the motor
    private final TalonFX armMotor = new TalonFX(16, "rio");

    public ArmSubsystem() {
       TalonFXConfiguration config = new TalonFXConfiguration();
        // Brakes when there is no output
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Feedforward (needs tuning)
        config.Slot0.kG = 0.45;        
        config.Slot0.kS = 0.45;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static; //ArmCosine?
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

    // Moves arm up one position
    public Command setUp(){
        if (pos >= positions.length - 1){
            return null;
        }
        else{
            pos += 1;
            return setPosition(pos);
        }
    }
    // Moves arm down one position
    public Command setDown(){
        if (pos <= 0){
            return null;
        }
        else{
            pos -= 1;
            return setPosition(pos);
        }
    }

    // Moves the arm to the position based on the list
    public Command setPosition(int targetPos){
        pos = targetPos;
        SmartDashboard.putNumber("Arm Position Index", pos);
        return setCloseLoop(() -> positions[targetPos]);
    }

    // Holds the position of the arm at a certain rotation
    // The rio will assume position 0 is when you click start, so always reset the arm to the bottom
    public Command setCloseLoop(DoubleSupplier position) {
        return Commands.run(() -> armMotor.setPosition(position.getAsDouble()), this);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Rotations", armMotor.getPosition().getValueAsDouble());
    }
}