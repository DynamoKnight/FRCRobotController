    package frc.robot.subsystems;

    import edu.wpi.first.wpilibj.Timer;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.Commands;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

    import com.revrobotics.spark.SparkLowLevel.MotorType;
    import com.revrobotics.spark.SparkMax;

    public class DumpRollerSubsystem extends SubsystemBase{
        
        // Initializes the motor
        SparkMax coralMotor = new SparkMax(2, MotorType.kBrushless);
        Timer a_timer = new Timer();
        // Indicates if the launcher is in action
        Boolean isRunning = false;
        // Rotation power
        double power = 0.2;

        // Initializes the motors and controller
        public DumpRollerSubsystem() {
            
        }

        // Outtakes the coral
        public Command dropCoral(double voltage){
            return Commands.run(() -> coralMotor.set(voltage), this);
        }
        
        // Holds the motor
        public Command keepCoral(){
            return Commands.run(() -> coralMotor.set(0), this);
        }

        @Override
        public void periodic() {
            
        }
    }
