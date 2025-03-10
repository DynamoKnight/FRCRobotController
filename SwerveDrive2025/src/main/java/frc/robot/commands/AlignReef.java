package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignReef extends Command{

    // Subsystems from RobotContainer
    private LimelightSubsystem limelight;
    private CommandSwerveDrivetrain drivetrain;
    private RobotContainer robotContainer;

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
    private final double speed = 1.0;
    // The speed (rad/s) to rotate to position
    private final double rotationSpeed = 0.75;
    // The tolerance before stopping align (meters)
    private final double positionTolerance = 0.01;
    // The tolerance for yaw alignment (radians)
    private final double yawTolerance = Math.PI / 32;
    // Indicates if alignment uses PID Control
    private final boolean usingPID = false;

    // Indicates the Left or Right side of reef
    ReefPos reefPos;
    // X and Y Offset from the April Tag (Default: Reef)
    private double offsetX = 0;
    private double offsetY = 0;
    // Targetted Tag ID
    private int tID;
    // Indicates if tag was detected
    private boolean tagDetected;
    // April Tags
    AprilTagFieldLayout aprilTagMap;

    // CONSTRUCTOR
    public AlignReef(RobotContainer robotContainer, ReefPos reefPos){
        this.robotContainer = robotContainer;
        this.drivetrain = robotContainer.drivetrain;
        this.limelight = robotContainer.limelight;

        this.reefPos = reefPos;
        // -180 and 180 degrees are the same point, so its continuous
        pidRotate.enableContinuousInput(-Math.PI, Math.PI);

        // Added Logging
        System.out.println("AlignReef command created for " + reefPos + " position");
        Logger.recordOutput("Reefscape/AlignReef/ReefPosition", reefPos.toString());
    }  
    
    @Override
    public void initialize(){
        // Starts timer
        timer.restart();
        Logger.recordOutput("Reefscape/AlignReef/CommandStarted", true);
        Logger.recordOutput("Reefscape/AlignReef/StartTime", timer.get());
        
        // Gets the tag ID that is being targeted
        tID = limelight.getTid();
        Logger.recordOutput("Reefscape/AlignReef/TargetTagID", tID);
        
        double[] aprilTagList = Constants.AprilTagMaps.aprilTagMap.get(tID);
        // Checks if the tag exists within the list of all tags
        if (aprilTagList == null) {
            System.out.println("Error: Target pose array is null for Tag ID: " + tID);
            Logger.recordOutput("Reefscape/AlignReef/Error", "Target pose array is null for Tag ID: " + tID);
            // Command is useless, thus it will end
            tagDetected = false;
            return;
        }
        Pose2d aprilTagPose = new Pose2d(aprilTagList[0] * Constants.inToM, aprilTagList[1] * Constants.inToM, new Rotation2d(aprilTagList[3] * Math.PI / 180));
        Logger.recordOutput("Reefscape/AlignReef/AprilTagPose", aprilTagPose);
        /*
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
        Optional<Pose3d> aprilTagOptional = aprilTagMap.getTagPose(tID);
        
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
        */
        tagDetected = true;
        Logger.recordOutput("Reefscape/AlignReef/TagDetected", tagDetected);
        
        // Reef Offset Positions - log the chosen offsets
        if (Constants.contains(new double[]{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}, tID)){
            if (reefPos == ReefPos.LEFT){
                System.out.println("left");
                offsetX = -0.4524;
                offsetY = 0.13;
            }
            else if (reefPos == ReefPos.RIGHT){
                System.out.println("right");
                offsetX = -0.4524;
                offsetY = -0.23;
            }
            Logger.recordOutput("Reefscape/AlignReef/OffsetX", offsetX);
            Logger.recordOutput("Reefscape/AlignReef/OffsetY", offsetY);
        }
        
        // The target rotation of the robot is opposite of the april tag's rotation
        double targetRotation = aprilTagPose.getRotation().getRadians() - Math.PI;
        // AngleModulus normalizes the difference to always take the shortest path
        targetRotation = MathUtil.angleModulus(targetRotation);
        Logger.recordOutput("Reefscape/AlignReef/TargetRotation", targetRotation);

        // Log offsets after rotation calculations
        double newOffsetX = (offsetX * Math.cos(targetRotation)) - (offsetY * Math.sin(targetRotation));
        double newOffsetY = (offsetX * Math.sin(targetRotation)) + ((offsetY * Math.cos(targetRotation)));
        Logger.recordOutput("Reefscape/AlignReef/RotatedOffsetX", newOffsetX);
        Logger.recordOutput("Reefscape/AlignReef/RotatedOffsetY", newOffsetY);
        
        // Creates a Pose2d for the target position
        targetPose = new Pose2d(aprilTagPose.getX() + newOffsetX, aprilTagPose.getY() + newOffsetY, new Rotation2d(targetRotation));
        Logger.recordOutput("Reefscape/AlignReef/TargetPose", targetPose);

        // Sets the destination to go to for the PID
        pidX.setSetpoint(targetPose.getX());
        pidY.setSetpoint(targetPose.getY());
        // Converts to radians
        pidRotate.setSetpoint(targetPose.getRotation().getRadians());
        // Prints Target Pose
        System.out.println("Target Pose2d: " + targetPose.getX() + ", " + targetPose.getY() + ", " + targetPose.getRotation().getDegrees());
        robotContainer.MaxSpeed = (TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    }

    // Called every 20ms to perform actions of Command
    @Override
    public void execute(){
        // Gets current robot Pose2d
        Pose2d currentPose = drivetrain.getState().Pose;
        Logger.recordOutput("Reefscape/AlignReef/CurrentPose", currentPose);
        
        // If no tag was detected, then Command wont execute
        if (!tagDetected){
            Logger.recordOutput("Reefscape/AlignReef/ExecuteSkipped", true);
            return;
        }
        Logger.recordOutput("Reefscape/AlignReef/ExecuteTime", timer.get());
        
        // List of X, Y, Yaw velocities to go to target pose
        double[] velocities;
        
        // PID Alignment
        if (usingPID){
            double yawError = MathUtil.angleModulus(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());
            Logger.recordOutput("Reefscape/AlignReef/YawError", yawError);
            
            // Rotates first
            if (Math.abs(yawError) > yawTolerance){
                Logger.recordOutput("Reefscape/AlignReef/Phase", "Rotation (PID)");
                velocities = calculateRotationalErrorPID(currentPose);
            }
            else{
                Logger.recordOutput("Reefscape/AlignReef/Phase", "Translation (PID)");
                velocities = calculateErrorPID(currentPose);
            }
        }
        // Regular Alignment
        else{
            double yawError = MathUtil.angleModulus(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());
            Logger.recordOutput("Reefscape/AlignReef/YawError", yawError);
            
            // Rotates first
            if (Math.abs(yawError) > yawTolerance){
                Logger.recordOutput("Reefscape/AlignReef/Phase", "Rotation");
                velocities = rotate();
            }
            // Moves, rotation is already done
            else{
                Logger.recordOutput("Reefscape/AlignReef/Phase", "Translation");
                velocities = calculateError(currentPose);
            }
        }
        
        // Logs values
        SmartDashboard.putNumberArray("Target Pose", new double[]{targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()});
        SmartDashboard.putNumberArray("Current Pose", new double[]{currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees()});
        SmartDashboard.putNumberArray("Target Vector", new double[]{velocities[0], velocities[1], velocities[2]});
        Logger.recordOutput("Reefscape/AlignReef/Velocities", velocities);

    // Log velocity command and direction flip for certain tags 
    boolean isFlippingDirection = Constants.contains(new double[]{6, 7, 8, 9, 10, 11}, tID);
    Logger.recordOutput("Reefscape/AlignReef/FlippedDirection", isFlippingDirection);

        // Moves the drivetrain
        if (isFlippingDirection){
            drivetrain.setControl(driveRequest.withVelocityX(-velocities[0]).withVelocityY(-velocities[1]).withRotationalRate(velocities[2]));
        }
        else{
            drivetrain.setControl(driveRequest.withVelocityX(velocities[0]).withVelocityY(velocities[1]).withRotationalRate(velocities[2]));
        }

    }

    // Calculates the needed velocities to get to the target pose
    public double[] calculateError(Pose2d currentPose){
         // Finds the translation difference (X2-X1, Y2-Y1) between the current and target pose
         Translation2d error = targetPose.getTranslation().minus(currentPose.getTranslation());
         // Finds the hypotenuse distance to the desired point
         double distance = error.getNorm();
         Logger.recordOutput("Reefscape/AlignReef/TranslationError", new double[]{error.getX(), error.getY()});
         Logger.recordOutput("Reefscape/AlignReef/Distance", distance);
         // This gets the robots current rotation (rad)
         // AngleModulus normalizes the difference to always take the shortest path
         double yawError = MathUtil.angleModulus(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());
         Logger.recordOutput("Reefscape/AlignReef/YawError", yawError);

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
        double[] result = new double[]{velocityX, velocityY, 0};
        Logger.recordOutput("Reefscape/AlignReef/CalculatedVelocities", result);
        return result;
    }   

    // Calculates the needed velocities to get to the target pose with PID
    private double[] calculateErrorPID(Pose2d currentPose){
        // Calculate the power for X direction and clamp it between -1 and 1
        double velocityX = pidX.calculate(currentPose.getX());
        velocityX = MathUtil.clamp(velocityX, -speed, speed);
        
        // Calculate the power for Y direction and clamp it between -1 and 1
        double velocityY = pidY.calculate(currentPose.getY());
        velocityY = MathUtil.clamp(velocityY, -speed, speed);

        Logger.recordOutput("Reefscape/Limelight/x error", pidX.getError());
        Logger.recordOutput("Reefscape/Limelight/y error", pidY.getError());
        Logger.recordOutput("Reefscape/Limelight/PIDOutputX", velocityX);
        Logger.recordOutput("Reefscape/Limelight/PIDOutputY", velocityY);

        // Returns the X, Y, Yaw powers
        double[] result = new double[]{velocityX, velocityY, 0};
        Logger.recordOutput("Reefscape/AlignReef/CalculatedPIDVelocities", result);
        return result;
    }

    private double[] calculateRotationalErrorPID(Pose2d currentPose){
        // Calculate the rotational power and clamp it between -2 and 2
        double velocityYaw = pidRotate.calculate(currentPose.getRotation().getRadians());
        velocityYaw = MathUtil.clamp(velocityYaw, -2, 2);
        Logger.recordOutput("Reefscape/AlignReef/RotationalError", pidRotate.getError());
        Logger.recordOutput("Reefscape/AlignReef/PIDOutputYaw", velocityYaw);
        
        // Returns the X, Y, Yaw powers
        double[] result = new double[]{0, 0, velocityYaw};
        Logger.recordOutput("Reefscape/AlignReef/CalculatedRotationPIDVelocities", result);
        return result;
    }

    // Returns the velocity for the yaw
    private double calculateYawVelocity(double yawError) {
        return Math.signum(yawError) * rotationSpeed;
    }

    // Returns the velocities needed to rotate to a specific angle relative to the robot
    public double[] rotate(){
        Pose2d currentPose = drivetrain.getState().Pose;
        double yawError = MathUtil.angleModulus(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());
        double velocityYaw;
        if (Math.abs(yawError) > yawTolerance) {
            velocityYaw = calculateYawVelocity(yawError);
        } 
        else {
            // Wont rotate if within tolerance
            velocityYaw = 0;
        }
        // Only rotation, no movement
        return new double[]{0, 0, velocityYaw};
    }

    // Called every 20ms to check if command is ended
    @Override
    public boolean isFinished(){
        // If a tag wasn't detected, command will end
        if (!tagDetected){
            //return true;
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
        robotContainer.MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 2;
        if (interrupted) {
            System.out.println("AlignReef interrupted.");
        } 
        else {
            System.out.println("AlignReef completed.");
        }
    }

}
