package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.Objects;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlignReef extends Command{

    // Subsystems from RobotContainer
    private LimelightSubsystem limelight;
    private CommandSwerveDrivetrain drivetrain;

    Timer timer = new Timer();

    /* ----- PIDs ----- */
    private PIDController pidX = new PIDController(3, 0, 0);
    private PIDController pidY = new PIDController(3, 0, 0);
    private PIDController pidRotate = new PIDController(4, 0, 0);

    // Creates a swerve request that specifies the robot to move FieldCentric
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // Creates a swerve request to stop all motion by setting velocities and rotational rate to 0
    SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);

    // The Desired position to go to
    private Pose2d targetPose;
    // The speed to move to position
    private final double speed = 0.3;
    // The speed (rad/s) to rotate to position
    private final double rotationSpeed = 0.5;
    // The tolerance before stopping align (meters)
    private final double positionTolerance = 0.1;
    // The tolerance for yaw alignment (degrees)
    private final double yawTolerance = 2.0;

    // CONSTRUCTOR
    public AlignReef(RobotContainer robotContainer){
        this.drivetrain = robotContainer.drivetrain;
        this.limelight = robotContainer.limelight;

        pidRotate.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void initialize(){
        timer.restart();
        // Gets the tag ID that is being targetted
        int tID = limelight.getTid();
        if (tID == null) {
            System.out.println("Error: Tag ID is null.");
            end(true); // End the command if tID is null
            return;
        // Gets the position of the april tag
        double[] targetPoseArray = Constants.AprilTagMaps.aprilTagMap.get(tID);
        // Checks if the tag exists within the list of all tags
        if (targetPoseArray == null) {
            System.out.println("Error: Target pose array is null for Tag ID: " + tID);
            end(true); // End the command if targetPoseArray is null
            return;
        }
        // Creates a Pose2d for the target position, converts inches to meters
        targetPose = new Pose2d(targetPoseArray[0] * Constants.inToM, targetPoseArray[1] * Constants.inToM, drivetrain.getState().Pose.getRotation());

        // Sets the destination to go to
        /*pidX.setSetpoint(1.52);
        pidY.setSetpoint(6.05);
        pidRotate.setSetpoint(122.0 * Math.PI / 180.0);*/

        SmartDashboard.putNumber("Tag ID", tID);
    }

    @Override
    public void execute(){
        Pose2d currentPose = drivetrain.getState().Pose;
        // Finds the translation difference (X2-X1, Y2-Y1) between the current and target pose
        Translation2d error = targetPose.getTranslation().minus(currentPose.getTranslation());
        // Finds the hypotenuse distance to the desired point
        double distance = error.getNorm();
        // This gets the hoz offset from the target
        double yawError = limelight.getTx();
        SmartDashboard.putNumber("Distance Error", distance);
        SmartDashboard.putNumber("X Error", error.getX());
        SmartDashboard.putNumber("Y Error", error.getY());
        // Intitializes rotation rates
        double velocityX = 0.0;
        double velocityY = 0.0;
        double rotationControl = 0.0;

        /*// Calculate the power for X direction and clamp it between -1 and 1
        double powerX = pidX.calculate(currentPose.getX());
        powerX = MathUtil.clamp(powerX, -1, 1);
        
        // Calculate the power for Y direction and clamp it between -1 and 1
        double powerY = pidY.calculate(pose.getY());
        powerY = MathUtil.clamp(powerY, -1, 1);

        Logger.recordOutput("Reefscape/Limelight/x error", pidX.getError());
        Logger.recordOutput("Reefscape/Limelight/y error", pidY.getError());

        // Calculate the rotational power and clamp it between -2 and 2
        double powerRotate = pidRotate.calculate(pose.getRotation().getRadians());
        powerRotate = MathUtil.clamp(powerRotate, -2, 2);

        // Moves the drivetrain
        drivetrain.setControl(driveRequest.withVelocityX(powerX).withVelocityY(powerY).withRotationalRate(powerRotate));
        */

        // Movement Correction
        if (distance > positionTolerance) {
            // Normalizes the error vector into a unit vector (value between -1 to 1) and applies the speed
            // The error vector represent both the direction and magnitude as the same. 
            velocityX = (error.getX() / distance) * speed;
            velocityY = (error.getY() / distance) * speed;
        } 
        else {
            // Wont move if within tolerance
            velocityX = 0;
            velocityY = 0;
        }
        // Rotational Correction
        if (Math.abs(yawError) > yawTolerance) {
            rotationControl = calculateRotationControl(yawError);
        } 
        else {
            // Wont rotate if within tolerance
            rotationControl = 0;
        }

        // Moves the drivetrain
        drivetrain.setControl(
            // Negative for some reason?
            driveRequest.withVelocityX(-velocityX).withVelocityY(-velocityY).withRotationalRate(-rotationControl)
        );
    }

    // Returns the direction of the rotation speed
    private double calculateRotationControl(double yawError) {
        return Math.signum(yawError) * rotationSpeed;
    }

    @Override
    public boolean isFinished(){
        Pose2d currentPose = drivetrain.getState().Pose;
        double distance = targetPose.getTranslation().getDistance(currentPose.getTranslation());
        // This gets the hoz offset from the target
        double yawError = limelight.getTx();

        // Ends once robot is within tolerance
        return distance <= positionTolerance && Math.abs(yawError) <= yawTolerance;
    }

    @Override
    public void end(boolean interrupted){
        // Ensures stop
        drivetrain.setControl(stop);

        if (interrupted) {
            System.out.println("AlignReef interrupted.");
        } 
        else {
            System.out.println("AlignReef completed.");
        }
    }

}
