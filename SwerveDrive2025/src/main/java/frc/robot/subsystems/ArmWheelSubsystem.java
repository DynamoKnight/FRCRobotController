package frc.robot.subsystems;

import java.time.Period;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmWheelSubsystem extends SubsystemBase{
    
    // The speed of the wheels
    private double spinSpeed = 0.5;
    //SparkMax spinMotor = new SparkMax(4, MotorType.kBrushless);
    TalonFX spinMotor = new TalonFX(18);
    public double velocity;

    public ArmWheelSubsystem(){

    }

    @Override
    public void periodic() {
        velocity = getVelocity();
        SmartDashboard.putNumber("intake velocity", velocity);
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

    // Returns the arm motors velocity
    public double getVelocity() {
        return spinMotor.getVelocity().getValueAsDouble();
        //return spinMotor.getEncoder().getVelocity();
    }

}