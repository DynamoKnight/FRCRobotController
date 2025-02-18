package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class LimelightSubsystem extends SubsystemBase {
    // Limelight Data table
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    // Basic targeting data

    // The ID of the targetted AprilTag
    double tid = LimelightHelpers.getFiducialID("limelight");
    // Horizontal offset from crosshair to target in degrees
    NetworkTableEntry tx = limelight.getEntry("tx");
    // Vertical offset from crosshair to target in degrees
    NetworkTableEntry ty = limelight.getEntry("ty");
    // Target area (0% to 100% of image)
    NetworkTableEntry ta = limelight.getEntry("ta");

    // A list X, Y, Z, Roll, Pitch, Yaw
    NetworkTableEntry botPose = limelight.getEntry("botpose_targetspace");
    NetworkTableEntry botPoseFieldBlue = limelight.getEntry("botpose_wpiblue");
    NetworkTableEntry activePipeline = limelight.getEntry("getpipe");
    NetworkTableEntry pipelineToSet = limelight.getEntry("pipeline");

    // Do you have a valid target?
    boolean hasTarget = LimelightHelpers.getTV("limelight");
    // Horizontal offset from principal pixel/point to target in degrees
    NetworkTableEntry txnc = limelight.getEntry("txnc");
    // Vertical offset from principal pixel/point to target in degrees
    NetworkTableEntry tync = limelight.getEntry("tync");

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
        // If Limelight is in use
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

            SmartDashboard.putNumber("limelight tx", tx.getDouble(0));
            SmartDashboard.putNumber("limelight yaw", getYaw());
            SmartDashboard.putNumber("limelight ta", ta.getDouble(0));
        }
    }

    public double getYaw() {
        // getDoubleArray returns the botPose as a list of doubles
        // The yaw is in the 4th element
        // The parameter is what it returns if nothing is found, in this case an empty list
        return botPose.getDoubleArray(new double[6])[4];
    }

    
    
}