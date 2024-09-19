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
    Servo servoThrower = new Servo(1);

    CommandXboxController controller;
    Timer a_timer = new Timer();
    Boolean isRunning = false;

    double lowPower = 1;
    double upPower = 1;

    //takes in controller, inverts upper and lower
    public LauncherSubsystem(CommandXboxController controller) {
        this.controller = controller;
        servoThrower.set(0);
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
        // Intakes on A button
        if (controller.a().getAsBoolean()) {
            upperMotor.setVoltage(-4);
            lowerMotor.setVoltage(-4);
        }
        // Left trigger starts the outtake to the top
        else if (controller.getLeftTriggerAxis() > 0.5 && !isRunning) {
            // Starts timer and indicates isRunning
            isRunning = true;
            // Resets the timer to 0
            a_timer.reset();
            a_timer.start();
            // Sets power
            upPower = 1;
            lowPower = 0.8;
        } 
        // Down d-pad starts the outtake to the bottom
        else if (controller.getHID().getPOV() == 180 && !isRunning) {
            // Starts timer and indicates isRunning
            isRunning = true;
            // Resets the timer to 0
            a_timer.reset();
            a_timer.start();
            // Sets power
            upPower = 0.25;
            lowPower = 0.25;
        } 
        // Runs while isRunning is true
        else if (isRunning) {
            // Motors Spin outwards while isRunning
            upperMotor.set(upPower);
            lowerMotor.set(lowPower);
            // After 2 seconds, the servo launches
            if (a_timer.get() > 2 && a_timer.get() <= 3) {
                servoThrower.set(0.5);
            }
            // After 3 seconds, stops the outtake
            if (a_timer.get() > 3) {
                isRunning = false;
                a_timer.stop();
                // Stops the motors and Resets the servo
                lowerMotor.set(0);
                upperMotor.set(0);
                servoThrower.set(0);
            }
        } 
        // If nothing is pressed, stop the motors and reset the servo
        else {
            lowerMotor.set(0);
            upperMotor.set(0);
            // Servo's default position
            servoThrower.set(0);
        }

        
    }


    
}
