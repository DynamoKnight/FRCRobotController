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
import edu.wpi.first.wpilibj.Timer;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  // CONSTANTS
  // Motor power From 0 to 1
  private double power = 0.5;
  // Keeps track of time since program started
  private final Timer m_timer = new Timer();

  // COMPONENTS
  // Motors
  private final CANSparkMax m_leftFront = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_leftBack = new CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightFront = new CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightBack = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless);
  // The Drivestation
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftFront::set, m_rightFront::set);
  private final XboxController m_controller = new XboxController(0);
 

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SendableRegistry.addChild(m_robotDrive, m_leftFront);
    SendableRegistry.addChild(m_robotDrive, m_leftBack);
    SendableRegistry.addChild(m_robotDrive, m_rightFront);
    SendableRegistry.addChild(m_robotDrive, m_rightBack);

    // Right gearbox faces opposite direciton
    m_rightFront.setInverted(true);
    
    // Follows after parent motor
    m_leftBack.follow(m_leftFront);
    m_rightBack.follow(m_rightFront);

    // Ensures robot doesn't coast when not moving
    m_rightFront.setIdleMode(IdleMode.kBrake);
    m_rightBack.setIdleMode(IdleMode.kBrake);
    m_leftFront.setIdleMode(IdleMode.kBrake);
    m_leftBack.setIdleMode(IdleMode.kBrake);
    
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // Left Trigger activates Speed mode
    if (m_controller.getLeftTriggerAxis() >= 0.01){
      power = 1.0;
    }
    else{
      power = 0.5;
    }
    // Deadzone for joysticks from 0 to 1
    m_robotDrive.setDeadband(0.05);
    // Arcade Drive: Left stick for movement speed, Right stick for turning
    m_robotDrive.arcadeDrive(-m_controller.getLeftY() * power, -m_controller.getRightX() * power);
    
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


}
