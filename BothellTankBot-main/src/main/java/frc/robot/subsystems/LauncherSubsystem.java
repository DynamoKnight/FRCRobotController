package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class LauncherSubsystem extends SubsystemBase {
    //shooter motors
    CANSparkFlex upperMotor = new CANSparkFlex(5, MotorType.kBrushless);
    CANSparkFlex lowerMotor = new CANSparkFlex(22, MotorType.kBrushless); // rename 6
    Servo servorThrower = new Servo(1);


    CommandXboxController controller;
    Timer a_timer = new Timer();

    //takes in controller, inverts upper and lower
    public LauncherSubsystem(CommandXboxController controller) {
        this.controller = controller;
        servorThrower.set(0.15);
        lowerMotor.setInverted(false);
        upperMotor.setInverted(false);
    }


    public void setUpperVoltage(double voltage) {
        upperMotor.setVoltage(voltage);
    }

    public void setLowerVoltage(double voltage) {
        lowerMotor.setVoltage(voltage);
    }

    //set voltage for both motors
    public void intake(double voltage) {
        upperMotor.setVoltage(-voltage);
        lowerMotor.setVoltage(-voltage);
    }

    @Override
    public void periodic() {
        System.out.println(a_timer.get());
        // Intakes on A button
        if (controller.a().getAsBoolean()) {
            upperMotor.setVoltage(-4);
            lowerMotor.setVoltage(-4);
        }
        // Motors Spin outwards on Left Trigger
        else if (controller.getLeftTriggerAxis() > 0.5){
            upperMotor.setVoltage(12);
            lowerMotor.setVoltage(12);
        }
        // Nothing is pressed don't move
        else {
            lowerMotor.setVoltage(0);
            upperMotor.setVoltage(0);
        }
        // Outtakes on Right Trigger
        if (controller.x().getAsBoolean()){
            servorThrower.set(0);
        }
        if (controller.b().getAsBoolean()){
            servorThrower.set(0.5);
        }
        
    }


    
}
