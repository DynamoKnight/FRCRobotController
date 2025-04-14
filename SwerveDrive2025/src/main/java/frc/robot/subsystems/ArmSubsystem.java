package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    TalonFX armMotor = new TalonFX(16);
    CANcoder encoder = new CANcoder(17);
    PIDController pid = new PIDController(2, 0, 0);

    public ArmSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        armMotor.getConfigurator().apply(config);

        pid.setSetpoint(-0.44);
    }

    public void setSetpoint(double setpoint) {
        pid.setSetpoint(setpoint);
    }

    @Override
    public void periodic() {
        double power = pid.calculate(encoder.getPosition().getValueAsDouble());
        armMotor.set(MathUtil.clamp(power, -0.2, 0.2));
        
        SmartDashboard.putNumber("Arm Encoder Position", encoder.getPosition().getValueAsDouble());
    }
}
