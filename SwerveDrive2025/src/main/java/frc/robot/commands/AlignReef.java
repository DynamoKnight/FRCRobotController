package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

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

    //private PIDController pidX = new PIDController(3, 0, 0);

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
        if (targetPoseArray == null) {
            System.out.println("Error: Target pose array is null for Tag ID: " + tID);
            end(true); // End the command if targetPoseArray is null
            return;
        // Creates a Pose2d for the target position, converts inches to meters
        targetPose = new Pose2d(targetPoseArray[0] * Constants.inToM, targetPoseArray[1] * Constants.inToM, drivetrain.getState().Pose.getRotation());

        SmartDashboard.putNumber("Tag ID", tID);

        //pidX.setSetpoint(1);
        //pidX.calculate(currentPose.getX())
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

        double velocityX = 0.0;
        double velocityY = 0.0;
        double rotationControl = 0.0;
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
