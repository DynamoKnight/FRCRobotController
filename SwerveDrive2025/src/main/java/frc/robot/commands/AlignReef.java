package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
    private final PIDController xController = new PIDController(6, 0, 0);
    private final PIDController yController = new PIDController(6, 0, 0);
    // Uses trapezoidal motion for smoother movement
    private final ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI)); // I set the max velocity to PI rad/s (It can rotate 180 deg in 1 sec)
    
    private final HolonomicDriveController controller;

    // Creates a swerve request that specifies the robot to move FieldCentric
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.Velocity); // Uses ClosedLoopVoltage for PID
    // Creates a swerve request to stop all motion by setting velocities and rotational rate to 0
    SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);

    // The Desired position to go to
    private Pose2d targetPose;
    // The speed to move to position
    private final double speed = 0.1;
    
    // Speed scaling parameters
    // The minimum speed factor (percentage of max speed)
    private final double minSpeedFactor = 0.0000001;
    // Scaling factor - higher values = faster slowdown as distance decreases
    private final double distanceScalingFactor = 12.0; 
    // The tolerance before stopping align (meters)
    private final double positionTolerance = 0.01;
    // The tolerance for yaw alignment (radians)
    private final double yawTolerance = Math.PI / 32;

    // Indicates the Left or Right side of reef
    ReefPos reefPos;
    // X and Y Offset from the April Tag (Default: Reef)
    private double offsetX = 0;
    private double offsetY = 0;
    // Targetted Tag ID
    private int tID;
    // Indicates if tag was detected
    private boolean tagDetected;
    // April Tags on the field
    AprilTagFieldLayout aprilTagMap;

    // CONSTRUCTOR
    public AlignReef(RobotContainer robotContainer, ReefPos reefPos){
        this.robotContainer = robotContainer;
        this.drivetrain = robotContainer.drivetrain;
        this.limelight = robotContainer.limelight;
        this.reefPos = reefPos;
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Create holonomic controller
        controller = new HolonomicDriveController(
            xController,
            yController,
            thetaController
        );
        
        // drivetrain is required for this command
        addRequirements(drivetrain);
        
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
        // Creates a Pose2D of the April Tag's position
        Pose2d aprilTagPose = new Pose2d(aprilTagList[0] * Constants.inToM, aprilTagList[1] * Constants.inToM, new Rotation2d(aprilTagList[3] * Math.PI / 180));
        // Tag is detected
        tagDetected = true;
        Logger.recordOutput("Reefscape/AlignReef/TagDetected", tagDetected);
        Logger.recordOutput("Reefscape/AlignReef/AprilTagPose", aprilTagPose);
        
        // Reef Offset Positions - log the chosen offsets
        if (Constants.contains(new double[]{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}, tID)){
            if (reefPos == ReefPos.LEFT){
                //System.out.println("left");
                offsetX = -0.41;
                offsetY = 0.13;
            }
            else if (reefPos == ReefPos.RIGHT){
                //System.out.println("right");
                offsetX = -0.41;
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
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        thetaController.setGoal(targetPose.getRotation().getRadians());
    }

    // Called every 20ms to perform actions of Command
    @Override
    public void execute() {
        // If no tag was detected, then Command won't execute
        if (!tagDetected) {
            Logger.recordOutput("Reefscape/AlignReef/ExecuteSkipped", true);
            return;
        }
        
        // Gets current robot Pose2d
        Pose2d currentPose = drivetrain.getState().Pose;
        Logger.recordOutput("Reefscape/AlignReef/CurrentPose", currentPose);
        Logger.recordOutput("Reefscape/AlignReef/ExecuteTime", timer.get());
        
        // Calculate distance to target for speed scaling
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        Logger.recordOutput("Reefscape/AlignReef/Distance", distance);
        
        /*double speedFactor = 1.0 / (1.0 + (distanceScalingFactor * distance));
        speedFactor = 1.0 - speedFactor;*/
        double slowdownStartDistance = 3.0;
        double speedFactor = Math.min(1.0, distance / slowdownStartDistance);
        
        // Ensure we don't go below minimum speed
        speedFactor = Math.max(minSpeedFactor, speedFactor);
        double scaledSpeed = speed * speedFactor;
        
        Logger.recordOutput("Reefscape/AlignReef/SpeedFactor", speedFactor);
        Logger.recordOutput("Reefscape/AlignReef/ScaledSpeed", scaledSpeed);
        
        // Calculates the chassis speeds required to move the robot to the target pose
        ChassisSpeeds speeds = controller.calculate(
            currentPose,
            targetPose,
            scaledSpeed, // Use scaled speed based on distance
            targetPose.getRotation()
        );
        
        // Direction flipping for red side
        boolean isFlippingDirection = Constants.contains(new double[]{6, 7, 8, 9, 10, 11}, tID);
        Logger.recordOutput("Reefscape/AlignReef/FlippedDirection", isFlippingDirection);
        
        if (isFlippingDirection) {
            drivetrain.setControl(driveRequest
                .withVelocityX(-speeds.vxMetersPerSecond)
                .withVelocityY(-speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond));
        } else {
            drivetrain.setControl(driveRequest
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond));
        }
        
        // Log velocities and errors
        Logger.recordOutput("Reefscape/AlignReef/Velocities", new double[]{speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond});
        Logger.recordOutput("Reefscape/Limelight/x error", xController.getPositionError());
        Logger.recordOutput("Reefscape/Limelight/y error", yController.getPositionError());
        Logger.recordOutput("Reefscape/AlignReef/RotationalError", thetaController.getPositionError());
    }

    // Called every 20ms to check if command is ended
    @Override
    public boolean isFinished() {
        // If a tag wasn't detected, command will end
        if (!tagDetected) {
            return true;
        }
        
        // Check if we've reached the target position and orientation
        Pose2d currentPose = drivetrain.getState().Pose;
        if (currentPose != null && targetPose != null) {
            double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double yawError = MathUtil.angleModulus(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());
            
            // End command if we're within tolerance
            if (distance <= positionTolerance && Math.abs(yawError) <= yawTolerance) {
                Logger.recordOutput("Reefscape/AlignReef/TargetReached", true);
                return true;
            }
        }
        
        return super.isFinished();
    }

    // Called once Command ends
    @Override
    public void end(boolean interrupted) {
        // Ensures drivetrain stop
        drivetrain.setControl(stop);
        robotContainer.MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7;
        if (interrupted) {
            System.out.println("AlignReef interrupted.");
        } 
        else {
            System.out.println("AlignReef completed.");
        }
        robotContainer.joystick.setRumble(RumbleType.kBothRumble, 0.2);
    }
}