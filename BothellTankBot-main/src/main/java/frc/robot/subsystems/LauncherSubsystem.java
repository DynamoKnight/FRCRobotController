package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class LauncherSubsystem extends SubsystemBase {
    //shooter motors
    CANSparkFlex upperMotor = new CANSparkFlex(5, MotorType.kBrushless);
    CANSparkFlex lowerMotor = new CANSparkFlex(22, MotorType.kBrushless); // rename 6

    CommandXboxController controller;
    Timer a_timer = new Timer();

    //takes in controller, inverts upper and lower
    public LauncherSubsystem(CommandXboxController controller) {
        this.controller = controller;
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
            upperMotor.setVoltage(-2);
            lowerMotor.setVoltage(-2);
        // Y button spins the top wheel so that it can get to full power before shooting
        } else if (controller.y().getAsBoolean()) {
            upperMotor.setVoltage(12);
            a_timer.restart();
            if (a_timer.get() > 2){
                lowerMotor.setVoltage(12);
            }
            
        } else if (controller.b().getAsBoolean()) {
            lowerMotor.setVoltage(12);
        } else if (controller.x().getAsBoolean()) {
            upperMotor.setVoltage(12);

        }else {
            //nothing is pressed, dont move
            lowerMotor.setVoltage(0);
            upperMotor.setVoltage(0);
        }
        
    }


    
}
