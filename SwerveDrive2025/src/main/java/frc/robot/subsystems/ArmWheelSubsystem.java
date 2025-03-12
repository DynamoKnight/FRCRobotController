package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmWheelSubsystem extends SubsystemBase{
    
    // The speed of the wheels
    private double spinSpeed = 0.5;
    SparkMax spinMotor = new SparkMax(4, MotorType.kBrushless);

    public ArmWheelSubsystem(){

    }
    
    // Command to move the wheels clockwise
    public Command clockwiseWheels(){
        return Commands.run(() -> spinMotor.set(spinSpeed),this);
    }

    // Command to move the wheels counter clockwise
    public Command counterClockwiseWheels(){
        return Commands.run(() -> spinMotor.set(-spinSpeed),this);
    }

    // Command to stop the wheels and hold at position
    public Command stopWheels(){
        return Commands.run(() -> spinMotor.set(0),this);
    }

    @Override
    public void periodic(){
        
    }
}