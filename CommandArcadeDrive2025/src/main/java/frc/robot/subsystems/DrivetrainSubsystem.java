// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  
  // The 4 motors of the base
  SparkMax frontLeftMotor = new SparkMax(Constants.Drivetrain.FRONT_LEFT_ID, MotorType.kBrushless);
  SparkMax backLeftMotor = new SparkMax(Constants.Drivetrain.BACK_LEFT_ID, MotorType.kBrushless);
  SparkMax frontRightMotor = new SparkMax(Constants.Drivetrain.FRONT_RIGHT_ID, MotorType.kBrushless);
  SparkMax backRightMotor = new SparkMax(Constants.Drivetrain.BACK_RIGHT_ID, MotorType.kBrushless);
  // The encoders of the main motors
  RelativeEncoder leftEncoder = frontLeftMotor.getEncoder();
  RelativeEncoder rightEncoder = frontRightMotor.getEncoder();
  // The gyroscope
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  // Power for movement and turn
  public double movePower = Constants.Drivetrain.SLOW_MOVE_POWER;
  public double turnPower = Constants.Drivetrain.SLOW_TURN_POWER;
  // Needed to track robot's position
  private final DifferentialDriveOdometry odometry;
  // Needed to track robot's velocity
  private final DifferentialDriveKinematics kinematics =
    new DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH_METERS);
  // PID Controllers for left and right side
  PIDController leftPID = new PIDController(1, 0, 0);
  PIDController rightPID = new PIDController(1, 0, 0);
  // The current velocity of the motors
  double currentLeftVelocity;
  double currentRightVelocity;

  public DrivetrainSubsystem() {
    // Creates a configuration to apply to motors
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    // Sets to brake when robot isn't given power
    motorConfig.idleMode(IdleMode.kBrake);
    // Reset mode resets the already saved flash settings to default values, Persist mode saves current settings across cycles
    frontLeftMotor.configure(motorConfig.inverted(true), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    backLeftMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    frontRightMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    backRightMotor.configure(motorConfig.inverted(true), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Initializes the robot's odometry based on the gyroscope and encoder positions
    odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(getHeading()),
      encoderToMeters(leftEncoder.getPosition()),
      encoderToMeters(rightEncoder.getPosition())
    );

    configureAutoBuilder();
  }

  /** Configures the AutoBuilder */
  private void configureAutoBuilder() {
    try{
      // Load the RobotConfig from the GUI settings. You should probably
      // store this in your Constants file
      var config = RobotConfig.fromGUISettings();
      // Configure AutoBuilder last
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Updates the odometry
    odometry.update(
      Rotation2d.fromDegrees(getHeading()),
      encoderToMeters(leftEncoder.getPosition()),
      encoderToMeters(rightEncoder.getPosition())
    );
    // Updates the velocity of the left and right motors
    currentLeftVelocity = encoderToVelocity(leftEncoder.getPosition());
    currentRightVelocity = encoderToVelocity(rightEncoder.getPosition());
  }
  

  /** Tank drive with left and right side power. */
  public void tankDrive(double left, double right) {
    frontLeftMotor.set(left);
    backLeftMotor.set(left);
    frontRightMotor.set(right);
    backRightMotor.set(right);
  }

  /**
   * Moves the robot with same side power.
   * @param power Power to move.
   */
  public void moveTank(double power) {
    frontLeftMotor.set(power);
    backLeftMotor.set(power);
    frontRightMotor.set(power);
    backRightMotor.set(power);
  }

  /**
   * Turns the robot by spinning the motors in opposite directions.
   * @param power Power to turn (positive for clockwise, negative for counterclockwise).
   */
  public void turnTank(double power) {
    frontLeftMotor.set(power);
    backLeftMotor.set(power);
    frontRightMotor.set(-power);
    backRightMotor.set(-power);
  }

  /** Stops all motors. */
  public void stop() {
    tankDrive(0, 0);
  }

  /** Sets the drive speed to fast. */
  public Command driveFast(){
    return runOnce(
      () -> {
        movePower = Constants.Drivetrain.FAST_MOVE_POWER;
        turnPower = Constants.Drivetrain.FAST_TURN_POWER;
    });
  }

  /** Sets the drive speed to slow. */
  public Command driveSlow(){
    return runOnce(
      () -> {
        movePower = Constants.Drivetrain.SLOW_MOVE_POWER;
        turnPower = Constants.Drivetrain.SLOW_TURN_POWER;
    });
  }

  /** Resets the encoder values. */
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /**
   * @return the average distance traveled by both sides (meters).
   */
  public double getDistanceTraveled() {
    double leftMeters = encoderToMeters(leftEncoder.getPosition());
    double rightMeters = encoderToMeters(rightEncoder.getPosition());
    return (leftMeters + rightMeters) / 2.0;
  }

  /**
   * @param rotations the number of motor rotations.
   * @return the number of meters from encoder rotations.
   */
  private double encoderToMeters(double rotations) {
    return rotations * Constants.Drivetrain.ENCODERS_TO_METERS;
  }

  /**
   * @param rotations the number of motor rotations.
   * @return the velocity of the motor from rotations.
   */
  private double encoderToVelocity(double rotations) {
    return encoderToMeters(rotations) / 60.0;
  }
  
  /** Moves the robot to a relative set position.
   * @param distanceMeters the distance to travel to.
   * @param power the speed & direction of the robot.
   */
  public Command driveToDistance(double distanceMeters, double power) {
    return runOnce(() -> resetEncoders())
    .andThen(run(() -> moveTank(power))
    .until(() -> getDistanceTraveled() >= distanceMeters)
    .finallyDo(() -> stop()));
  }

  /** Resets the encoder values. */
  public void resetGyro() {
    gyro.reset();
  }

  /** Returns the heading in degrees (CCW positive). */
  public double getHeading() {
    return gyro.getAngle();
  }

  /** Turns the robot to a relative set angle.
   * @param targetAngle the angle to turn to.
   * @param power the speed * direction of the robot.
   */
  public Command turnToAngle(double targetAngle, double power) {
    return runOnce(() -> resetGyro())
    .andThen(run(() -> turnTank(power))
    .until(() ->  Math.abs(getHeading() - targetAngle) < 2.0) // error of 2
    .finallyDo(() -> stop()));
  }

  // AUTO BUILDER METHODS
  /** Returns the robot’s current pose on the field. */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Resets odometry to a given pose. */
  public void resetPose(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        encoderToMeters(leftEncoder.getPosition()),
        encoderToMeters(rightEncoder.getPosition()),
        pose
    );
  }

  /** Returns a ChassisSpeed object of the robot speeds. */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
        encoderToVelocity(leftEncoder.getPosition()),
        encoderToVelocity(rightEncoder.getPosition())
    );
    return kinematics.toChassisSpeeds(wheelSpeeds);
  }

  /** Drives the robot using ChassisSpeeds. */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    // Convert ChassisSpeeds → wheel speeds
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    // The target speeds
    double leftVelocity = wheelSpeeds.leftMetersPerSecond;
    double rightVelocity = wheelSpeeds.rightMetersPerSecond;
    // Calculates the output for going from the current to the target velocity
    double leftPower = leftPID.calculate(currentLeftVelocity, leftVelocity);
    double rightPower = rightPID.calculate(currentRightVelocity, rightVelocity);
    tankDrive(leftPower, rightPower); 
  }

}
