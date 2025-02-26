package frc.robot.commands;

import java.io.IOException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignReef extends Command{

    // Subsystems from RobotContainer
    private LimelightSubsystem limelight;
    private CommandSwerveDrivetrain drivetrain;

    Timer timer = new Timer();

    /* ----- PIDs ----- */
    private PIDController pidX = new PIDController(1, 0, 0);
    private PIDController pidY = new PIDController(1, 0, 0);
    private PIDController pidRotate = new PIDController(2, 0, 0);

    // Creates a swerve request that specifies the robot to move FieldCentric
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // Creates a swerve request to stop all motion by setting velocities and rotational rate to 0
    SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);

    // The Desired position to go to
    private Pose2d targetPose;
    // The speed to move to position
    private final double speed = 1.0;
    // The speed (rad/s) to rotate to position
    private final double rotationSpeed = 0.5;
    // The tolerance before stopping align (meters)
    private final double positionTolerance = 0.01;
    // The tolerance for yaw alignment (radians)
    private final double yawTolerance = Math.PI / 64;
    // Indicates if alignment uses PID Control
    private final boolean usingPID = false;

    // Indicates the Left or Right side of reef
    ReefPos reefPos;
    // X and Y Offset from the April Tag (Default: Reef)
    private double offsetX = -0.3876;
    private double offsetY = -0.2759;
    // Targetted Tag ID
    private int tID;
    // Indicates if tag was detected
    private boolean tagDetected;
    // April Tags
    AprilTagFieldLayout aprilTagMap;

    // CONSTRUCTOR
    public AlignReef(RobotContainer robotContainer, ReefPos reefPos){
        this.drivetrain = robotContainer.drivetrain;
        this.limelight = robotContainer.limelight;

        this.reefPos = reefPos;
        // -180 and 180 degrees are the same point, so its continuous
        pidRotate.enableContinuousInput(-Math.PI, Math.PI);
    }     
    
    @Override
    public void initialize(){
        // Starts timer
        timer.restart();
        // Gets the tag ID that is being targeted
        tID = limelight.getTid();
        // Sets the April Tag Field Layout for Reefscape
        try {
            aprilTagMap = new AprilTagFieldLayout("SwerveDrive2025/2025-reefscape-welded.json");
        } 
        catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            System.out.println("Path not found for AprilTagFieldLayout!");
            tagDetected = false;
            return;
        }
        // Attempts to get the Pose3d of the april tag
        Optional<Pose3d> aprilTagOptional = aprilTagMap.getTagPose(18);
        Pose3d aprilTagPose;
        // If an object exists, then get the Pose3d
        if (aprilTagOptional.isPresent()){
            // Gets the position of the april tag as a Pose3d
            aprilTagPose = aprilTagOptional.get();
        }
        // Object didn't exist
        else{
            System.out.println("Error: Pose is null for Tag ID: " + tID);
            // Command is useless, thus it will end
            tagDetected = false;
            return;
        }
        tagDetected = true;
        // Reef Offset Positions
        if (Constants.contains(new double[]{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}, tID)){
            if (reefPos == ReefPos.LEFT){
                offsetX = -0.2124;
                offsetY = 0.1130;
            }
            else if (reefPos == ReefPos.RIGHT){
                offsetX = -0.2124;
                offsetY = -0.2841;
            }
        }
        // Creates a Rotation2D of the target rotation of the robot (radians)
        // The target rotation of the robot is opposite of the april tag's rotation
        Rotation2d yaw = new Rotation2d(aprilTagPose.getRotation().getAngle() - Math.PI);
        // Calculates offset based on robots rotation
        offsetX = (offsetX * Math.cos(yaw.getRadians())) - (offsetY * Math.sin(yaw.getRadians()));
        offsetY = (offsetX * Math.sin(yaw.getRadians())) + ((offsetY * Math.cos(yaw.getRadians())));
        // Creates a Pose2d for the target position
        targetPose = new Pose2d(aprilTagPose.getX() + offsetX, aprilTagPose.getY() + offsetY, yaw);

        // Sets the destination to go to for the PID
        pidX.setSetpoint(targetPose.getX());
        pidY.setSetpoint(targetPose.getY());
        // Converts to radians
        pidRotate.setSetpoint(targetPose.getRotation().getRadians());
        // Displays tID in AdvantageScope
        Logger.recordOutput("Reefscape/Limelight/Targetted tID", tID);
        // Displays Target Pose in AdvantageScope
        Logger.recordOutput("Reefscape/Limelight/Target Pose tID", targetPose.getX() + ", " + targetPose.getY() + ", " + targetPose.getRotation().getDegrees());
        
    }

    // Called every 20ms to perform actions of Command
    @Override
    public void execute(){
        // If no tag was detected, then Command wont execute
        if (!tagDetected){
            return;
        }        
        // List of X, Y, Yaw velocities to go to target pose
        double[] velocities;
        // PID Alignment
        if (usingPID){
            velocities = calculateErrorPID();
        }
        // Regular Alignment
        else{
            velocities = calculateError();
        }
        // Moves the drivetrain
        drivetrain.setControl(driveRequest.withVelocityX(velocities[0]).withVelocityY(velocities[1]).withRotationalRate(velocities[2]));
        // Displays Robot Target Vector in AdvantageScope
        Logger.recordOutput("Reefscape/Limelight/Target Pose tID", velocities[0] + ", " + velocities[1] + ", " + velocities[2]);

    }

    // Calculates the needed velocities to get to the target pose
    public double[] calculateError(){
        Pose2d currentPose = drivetrain.getState().Pose;

         // Finds the translation difference (X2-X1, Y2-Y1) between the current and target pose
         Translation2d error = targetPose.getTranslation().minus(currentPose.getTranslation());
         // Finds the hypotenuse distance to the desired point
         double distance = error.getNorm();
         // This gets the robots current rotation (rad)
         // AngleModulus normalizes the difference to always take the shortest path
         double yawError = MathUtil.angleModulus(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());
         
         // Displays Error in AdvantageScope
         Logger.recordOutput("Reefscape/Limelight/Error", "Distance: "+ -distance + ", Pose:" + error.getX() + ", " + error.getY() + ", " + (yawError * 180 / Math.PI));

         // Intitializes rotation rates
         double velocityX = 0.0;
         double velocityY = 0.0;
         double velocityYaw = 0.0;

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
            velocityYaw = calculateYawVelocity(yawError);
        } 
        else {
            // Wont rotate if within tolerance
            velocityYaw = 0;
        }
        // Returns the X, Y, Yaw powers
        return new double[]{velocityX, velocityY, velocityYaw};
    }

    // Calculates the needed velocities to get to the target pose with PID
    private double[] calculateErrorPID(){
        Pose2d currentPose = drivetrain.getState().Pose;

        // Calculate the power for X direction and clamp it between -1 and 1
        double velocityX = pidX.calculate(currentPose.getX());
        velocityX = MathUtil.clamp(velocityX, -speed, speed);
        
        // Calculate the power for Y direction and clamp it between -1 and 1
        double velocityY = pidY.calculate(currentPose.getY());
        velocityY = MathUtil.clamp(velocityY, -speed, speed);

        Logger.recordOutput("Reefscape/Limelight/x error", pidX.getError());
        Logger.recordOutput("Reefscape/Limelight/y error", pidY.getError());

        // Calculate the rotational power and clamp it between -2 and 2
        double velocityYaw = pidRotate.calculate(currentPose.getRotation().getRadians());
        velocityYaw = MathUtil.clamp(velocityYaw, -2, 2);

        // Returns the X, Y, Yaw powers
        return new double[]{velocityX, velocityY, velocityYaw};
    }

    // Returns the velocity for the yaw
    private double calculateYawVelocity(double yawError) {
        return Math.signum(yawError) * rotationSpeed;
    }

    // Called every 20ms to check if command is ended
    @Override
    public boolean isFinished(){
        // If a tag wasn't detected, command will end
        if (!tagDetected){
            return true;
        }
        // Without PID, it will check until the tolerance is reached
        if (!usingPID){
            Pose2d currentPose = drivetrain.getState().Pose;
            double distance = targetPose.getTranslation().getDistance(currentPose.getTranslation());
            // This gets the yaw error from the target
            double yawError = MathUtil.angleModulus(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

            // Ends once robot is within tolerance
            return distance <= positionTolerance && Math.abs(yawError) <= yawTolerance;
        }
        // PID will have its own tolerance check, so isFinished is unnecessary
        else{
            return super.isFinished();
        }
    }

    // Called once Command ends
    @Override
    public void end(boolean interrupted){
        // Ensures drivetrain stops
        drivetrain.setControl(stop);

        if (interrupted) {
            System.out.println("AlignReef interrupted.");
        } 
        else {
            System.out.println("AlignReef completed.");
        }
    }

}
