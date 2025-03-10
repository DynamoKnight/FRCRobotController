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

public class ArmWheelSubsystem extends SubsystemBase{
    

    private double spinSpeed = 0.5;
    SparkMax spinMotor = new SparkMax(4, MotorType.kBrushless);

    public ArmWheelSubsystem(){

    }
    
    public Command clockwiseWheels(){
        return Commands.run(() -> spinMotor.set(spinSpeed),this);
    }

    public Command counterClockwiseWheels(){
        return Commands.run(() -> spinMotor.set(-spinSpeed),this);
    }


    public Command stopWheels(){
        return Commands.run(() -> spinMotor.set(0),this);
    }

    @Override
    public void periodic(){
        
    }
}