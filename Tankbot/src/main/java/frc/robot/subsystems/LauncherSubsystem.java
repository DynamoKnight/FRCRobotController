package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class LauncherSubsystem extends SubsystemBase {
    // The launcher motors
    SparkFlex upperMotor = new SparkFlex(5, MotorType.kBrushless);
    SparkFlex lowerMotor = new SparkFlex(22, MotorType.kBrushless); // rename 6
    // Holds the object before moving it to the motors
    Servo servoThrower = new Servo(1);

    CommandXboxController controller;
    Timer a_timer = new Timer();
    // Indicates if the launcher is in action
    Boolean isRunning = false;

    // Power to launch to speaker
    public double spkrUpPower = 1;
    public double spkrLowPower = 0.8;
    // Power to launch to amp
    public double ampUpPower = 0.25;
    public double ampLowPower = 0.25;
    // Power for both motors to intake
    public double intakePower = 0.3;

    // Holds note down then up
    public double servoPos0 = 0.5;
    public double servoPos1 = 1;

    // Initializes the motors and controller
    public LauncherSubsystem(CommandXboxController controller) {
        this.controller = controller;
        servoThrower.set(servoPos0);
        // Configures motor
        SparkFlexConfig config = new SparkFlexConfig();
        config.inverted(false);
        lowerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        upperMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Sets the voltage from 0-12 of the upper motor
    public void setUpperVoltage(double voltage) {
        upperMotor.setVoltage(voltage);
    }
    // Sets the voltage from 0-12 of the lower motor
    public void setLowerVoltage(double voltage) {
        lowerMotor.setVoltage(voltage);
    }

    // Set power for both motors from 0-1
    public void intake(double power) {
        upperMotor.set(-power);
        lowerMotor.set(-power);
    }

    @Override
    public void periodic() {
        // Left trigger starts the outtake to the speaker
        if (controller.getLeftTriggerAxis() > 0.5) {
            upperMotor.set(spkrUpPower);
            lowerMotor.set(spkrLowPower);
        }
        // Down d-pad starts the outtake to the amp
        else if (controller.getHID().getPOV() == 180) {
            upperMotor.set(ampUpPower);
            lowerMotor.set(ampLowPower);
        }
        // Intakes on A button
        else if (controller.a().getAsBoolean()) {
            intake(intakePower);
        }
        // Dont move at all
        else{
            upperMotor.set(0);
            lowerMotor.set(0);
        }
        // Hold right trigger->servo launches
        if(controller.getRightTriggerAxis() > 0.5 && !isRunning){
            isRunning = true;
            startLaunch();
            servoThrower.set(servoPos1);
        }
        //if it is running, stop running after 1 sec
        if (isRunning)  {
            if(a_timer.get() > 1){
                servoThrower.set(servoPos0);
                isRunning = false;
                a_timer.stop();
            }

        }    
    }

    public void startLaunch() {
        // Resets the timer to 0
        a_timer.reset();
        a_timer.start();
        isRunning = true;
    }

    // Stops the motors and resets the servo to original posititon
    public void resetLauncher(){
        lowerMotor.set(0);
        upperMotor.set(0);
        servoThrower.set(servoPos0);
    }

    /**
     * Fires the note at a given power
     * @param upPower The power for the upper motor from 0-1
     * @param lowPower The power for the lower motor from 0-1
     */
    public void launch(double upPower, double lowPower){
        // Resets the timer to 0
        a_timer.reset();
        a_timer.start();
        // For the first 2 seconds, the motors gain speed
        while (a_timer.get() < 2){
            // Motors Spin outwards while isRunning
            upperMotor.set(upPower);
            lowerMotor.set(lowPower);
        }
        // After 2 seconds, the servo launches
        while (a_timer.get() > 2 && a_timer.get() <= 3){
            servoThrower.set(servoPos1);
        }
        // After 3 seconds, it stops the outtake
        isRunning = false;
        a_timer.stop();
        resetLauncher();
    }

    


    
}
