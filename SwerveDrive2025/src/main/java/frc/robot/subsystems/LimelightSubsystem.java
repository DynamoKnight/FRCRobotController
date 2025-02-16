package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class LimelightSubsystem extends SubsystemBase {
    // Basic targeting data
    double tx = LimelightHelpers.getTX("limelight");  // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY("limelight");  // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA("limelight");  // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV("limelight"); // Do you have a valid target?

    //NetworkTableEntry tid = limelight.getEntry("tid");
    double txnc = LimelightHelpers.getTXNC("limelight");  // Horizontal offset from principal pixel/point to target in degrees
    double tync = LimelightHelpers.getTYNC("limelight");  // Vertical  offset from principal pixel/point to target in degrees

    // Indicates if limelight is being used
    private final boolean kUseLimelight = false;
    // RobotContainer
    private final RobotContainer m_robotContainer;

    // CONSTRUCTOR
    public LimelightSubsystem(RobotContainer m_robotContainer) {
        // Switch to pipeline 0
        LimelightHelpers.setPipelineIndex("limelight", 0);
        // Sets LED settings
        LimelightHelpers.setLEDMode_PipelineControl("limelight");
        LimelightHelpers.setLEDMode_ForceOn("limelight");
        this.m_robotContainer = m_robotContainer;
    }

    @Override
    public void periodic() {
        if (kUseLimelight) {
            // Returns the robot's current state(position, orientation, and velocity)
            var driveState = m_robotContainer.drivetrain.getState();
            // Gets the heading/rotation of the robot in degrees
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            // Converts the robot's angular velocity from radians to rotations per second
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
    
            LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
            var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
                // Adds the limelight pose estimate to the drivetrain's odometry calculation
                m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
            }
        }
    }

    
    
}