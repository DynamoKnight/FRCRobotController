package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignReef extends Command{

    // Subsystems from RobotContainer
    private LimelightSubsystem limelight;
    private CommandSwerveDrivetrain drivetrain;

    Timer timer = new Timer();

    /* ----- PIDs ----- */
    private PIDController pidX = new PIDController(2, 0, 0);
    private PIDController pidY = new PIDController(2, 0, 0);
    private PIDController pidRotate = new PIDController(3, 0, 0);

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
    private final double positionTolerance = 0.05;
    // The tolerance for yaw alignment (radians)
    private final double yawTolerance = Math.PI / 128;
    // Indicates if alignment uses PID Control
    private final boolean usingPID = false;
    // X and Y Offset from the April Tag (Default: Reef)
    // April Tag position: (3.6576, 4.0259)
    // Previous Position 18: 3.27, 3.75
    private double offsetX = -0.3876;
    private double offsetY = -0.2759;

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
        // Gets the position of the april tag
        double[] targetPoseArray = Constants.AprilTagMaps.aprilTagMap.get(tID);
        // Checks if the tag exists within the list of all tags
        if (targetPoseArray == null) {
            System.out.println("Error: Target pose array is null for Tag ID: " + tID);
            end(true); // End the command if targetPoseArray is null
            return;
        }
        // Reef Offset Positions
        if (tID == 17 || tID == 18 || tID == 19){
            offsetX = -0.3876;
            offsetY = -0.2759;
        }
        // Creates a Pose2d for the target position]]\[]
        targetPose = new Pose2d(targetPoseArray[0] * Constants.inToM + offsetX, targetPoseArray[1] * Constants.inToM + offsetY, new Rotation2d((targetPoseArray[3] - 180) * Math.PI / 180));

        // Sets the destination to go to for the PID
        pidX.setSetpoint(targetPose.getX());
        pidY.setSetpoint(targetPose.getY());
        // Converts to radians
        pidRotate.setSetpoint(targetPose.getRotation().getRadians());

        SmartDashboard.putNumber("Tag ID", tID);
    }

    @Override
    public void execute(){
        // List of X, Y, Yaw velocities to go to target pose
        System.out.println(limelight.getTid());
        System.out.println(targetPose.getX() + " | " + targetPose.getY() + " | " + targetPose.getRotation().getRadians());
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
        drivetrain.setControl(driveRequest.withVelocityX(-velocities[0]).withVelocityY(-velocities[1]).withRotationalRate(-velocities[2]));
    }

    // Calculates the needed velocities to get to the target pose
    public double[] calculateError(){
        Pose2d currentPose = drivetrain.getState().Pose;

         // Finds the translation difference (X2-X1, Y2-Y1) between the current and target pose
         Translation2d error = targetPose.getTranslation().minus(currentPose.getTranslation());
         // Finds the hypotenuse distance to the desired point
         double distance = error.getNorm();
         // This gets the robots current rotation (rad)
         double yawError = currentPose.getRotation().getRadians();
         SmartDashboard.putNumber("Distance Error", distance);
         SmartDashboard.putNumber("X Error", error.getX());
         SmartDashboard.putNumber("Y Error", error.getY());

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
        velocityX = MathUtil.clamp(velocityX, -1, 1);
        velocityX += .1*Math.signum(velocityX);
        
        // Calculate the power for Y direction and clamp it between -1 and 1
        double velocityY = pidY.calculate(currentPose.getY());
        velocityY = MathUtil.clamp(velocityY, -1, 1);
        velocityY += .1*Math.signum(velocityY);

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

    @Override
    public boolean isFinished(){
        // Without PID, it needs to check until the tolerance is reached
        if (!usingPID){
            Pose2d currentPose = drivetrain.getState().Pose;
            double distance = targetPose.getTranslation().getDistance(currentPose.getTranslation());
            // This gets the hoz offset from the target
            double yawError = currentPose.getRotation().getRadians();

            // Ends once robot is within tolerance
            return distance <= positionTolerance && Math.abs(yawError) <= yawTolerance;
        }
        // PID will have its own tolerance check, so isFinished is unnecessary
        else{
            return super.isFinished();
        }
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
