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
import java.lang.Math;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

	// CONSTANTS
	// Motor power From 0 to 1
	private double power = 0.7;
	private double turnPower = 0.7;
	// The Pulses per revolution of the motor
	private double encoder_resolution = 42;
	// The Diameter of the wheel (inches)
	private double w_diameter = 6;
	// 8.45 revolutions of the motor is 1 revolution of the wheel
	private double m_w_ratio = 8.45;
	// Keeps track of time since program started
	private final Timer m_timer = new Timer();

	// COMPONENTS
	// Motors
	private final CANSparkMax m_leftFront = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless);
	private final CANSparkMax m_leftBack = new CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless);
	private final CANSparkMax m_rightFront = new CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless);
	private final CANSparkMax m_rightBack = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless);
	// The Drivestation
	//private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftFront::set, m_rightFront::set);
	private final XboxController m_controller = new XboxController(0);

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Adds a child to the parent object
		// SendableRegistry allows you to register sensors & actuators on the dashboard
		/*SendableRegistry.addChild(m_robotDrive, m_leftFront);
		SendableRegistry.addChild(m_robotDrive, m_leftBack);
		SendableRegistry.addChild(m_robotDrive, m_rightFront);
		SendableRegistry.addChild(m_robotDrive, m_rightBack);*/

		// Right gearbox faces opposite direciton
		m_leftFront.restoreFactoryDefaults();
		m_leftBack.restoreFactoryDefaults();
		m_rightFront.restoreFactoryDefaults();
		m_rightBack.restoreFactoryDefaults();

		m_rightFront.setInverted(true);
		m_rightBack.setInverted(true);

		// Follows after parent motor
		//m_leftBack.follow(m_leftFront);
		//m_rightBack.follow(m_rightFront);

		// Ensures robot doesn't coast when not moving
		m_leftFront.setIdleMode(IdleMode.kBrake);
		m_leftBack.setIdleMode(IdleMode.kBrake);
		m_rightFront.setIdleMode(IdleMode.kBrake);
		m_rightBack.setIdleMode(IdleMode.kCoast);

	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
		//m_leftFront.getEncoder().setPosition(0);
	}


	/** This function is called periodically during teleoperated mode. */
	@Override
	public void teleopPeriodic() {
		// Prints the position (number of revolutions) of the motor
		//System.out.println(m_leftFront.getEncoder().getPosition());

		// Left Trigger activates Speed mode
		if (m_controller.getLeftTriggerAxis() >= 0.01) {
			power = 1.0;
			turnPower = 1.0;
		} else {
			power = 0.7;
			turnPower = 0.7;
		}
		// Deadzone for joysticks, from 0 to 1
		//m_robotDrive.setDeadband(0.05);

		// Arcade Drive: Left stick for movement speed, Right stick for turning
		//m_robotDrive.arcadeDrive(-m_controller.getLeftY() * power, -m_controller.getRightX() * turnPower);
		double leftY = -m_controller.getLeftY();
		double rightX = m_controller.getRightX();
		// Deadband
		if (Math.abs(leftY) < 0.15){
			leftY = 0;
		}
		
		if (Math.abs(rightX) < 0.15){
			rightX = 0;
		}
		// Lessens power
		rightX *= 0.4;
		// Vectors
		double leftSide = leftY + rightX;
		double rightSide = leftY - rightX;

		m_leftFront.set(leftSide);
		m_leftBack.set(leftSide);
		m_rightFront.set(rightSide);
		m_rightBack.set(rightSide);

		// Tank drive: Left and Right movement is controlled independently
		// m_robotDrive.tankDrive(-m_controller.getY(), -m_controller.getY());

		// Curvature drive: Like Arcade Drive, but has a button for turning in-place.
		// m_robotDrive.curvatureDrive(-m_controller.getLeftY() * power,
		// -m_controller.getRightX() * power, m_controller.getButton(1));

	}

	/** This function is run once each time the robot enters autonomous mode. */
	@Override
	public void autonomousInit() {
		m_timer.restart();
		move(5);
		moveTo(2);
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		/*
		// Drive for 2 seconds
		if (m_timer.get() < 2) {
			// Drive forwards half speed, make sure to turn input squaring off
			m_robotDrive.arcadeDrive(0.1, 0.0, false);
		} else {
			m_robotDrive.stopMotor();
		}
		*/
	}

	/** This function is called once each time the robot enters test mode. */
	@Override
	public void testInit() {
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}

	/**
	 * Converts the target distance to ticks.
	 * @param inches is the displacement needed to travel.
	 * @return the number of ticks that the wheels have to move
	 */
	public double distanceToRevolutions(double inches) {
		// Circumference = Diameter * pi
		// Revolutions = Distance / Circumference
		// Number of Ticks = Encoder * Revolutions
		double revolutions = inches / (w_diameter * Math.PI);
		return revolutions;
	}


	/**
	 * Moves all motors to the desired position
	 * @param position is the revolutions from the original position
	 */
	public void moveTo(double position){
		// Needs to be in terms of wheel rotations
		position *= m_w_ratio;
		m_leftFront.getEncoder().setPosition(position);
		m_leftBack.getEncoder().setPosition(position);
		m_rightFront.getEncoder().setPosition(position);
		m_rightBack.getEncoder().setPosition(position);
	}

	/**
	 * Moves all motors a number of revolutions
	 * @param revolutions is the number of revolutions to go
	 */
	public void move(double revolutions){
		// Moves from the current position
		moveTo(m_leftFront.getEncoder().getPosition() + revolutions);
	}

	/**
	 * Turns the robot to the desired angle
	 * @param angle is the degrees from the original orientation
	 */
	public void turnTo(double angle) {
		
	}

	/**
	 * Turns the robot a number of degrees
	 * @param angle is the degrees to go
	 */
	public void turn(double angle) {
		
	}

}
