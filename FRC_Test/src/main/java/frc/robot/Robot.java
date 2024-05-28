// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_robotDrive;
  private XboxController controller;
 

  private final CANSparkMax m_rightFrontMotor = new CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightBackMotor = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_leftFrontMotor = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_leftBackMotor = new CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless);
  double power = 0.5;
 

  @Override
  public void robotInit() {
    SendableRegistry.addChild(m_robotDrive, m_leftFrontMotor);
    SendableRegistry.addChild(m_robotDrive, m_leftBackMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightFrontMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightBackMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    controller = new XboxController(0);
    m_robotDrive = new DifferentialDrive(m_leftFrontMotor::set, m_rightFrontMotor::set);
    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);

    // Ensures robot doesn't coast when not moving
    m_rightFrontMotor.setIdleMode(IdleMode.kBrake);
    m_rightBackMotor.setIdleMode(IdleMode.kBrake);
    m_leftFrontMotor.setIdleMode(IdleMode.kBrake);
    m_leftBackMotor.setIdleMode(IdleMode.kBrake);

    
    
  }

  @Override
  public void teleopPeriodic() {
    if (controller.getLeftTriggerAxis() >= 0.01){
      power = 1.0;
    }
    else{
      power = 0.5;
    }
    m_robotDrive.tankDrive(-MathUtil.applyDeadband(controller.getLeftY(), 0.05)*power, MathUtil.applyDeadband(controller.getLeftY(), 0.05)*power);
    //m_robotDrive.tankDrive(-controller.getLeftX()*power, controller.getRightX()*power);
    


    
  }
}
