package frc.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;

public class LimelightEx implements Sendable{
    private final String name;

    private LimelightEx(String name) {
        this.name=name;
    }

    public static LimelightEx create(String name) {
        return new LimelightEx(name);
    }

    /**
     * Sets the camera pose relative to the robot's center.
     * 
     * @param forward Forward offset in meters
     * @param side Side offset in meters
     * @param up Up offset in meters
     * @param roll Roll angle in degrees
     * @param pitch Pitch angle in degrees
     * @param yaw Yaw angle in degrees
     */
    public void setMountPose(double forward, double side, double up, double roll, double pitch, double yaw) {
        LimelightHelpers.setCameraPose_RobotSpace(name,forward,side,up,roll,pitch,yaw);
    }

    /** Enables standard side-by-side stream mode. */
    public LimelightEx withStreamMode_Standard() {
        LimelightHelpers.setStreamMode_Standard(name);
        return this;
    }

    /**
     * Sets the crop window for the camera. The crop window in the UI must be completely open.
     *
     * @param cropXMin Minimum X value (-1 to 1)
     * @param cropXMax Maximum X value (-1 to 1)
     * @param cropYMin Minimum Y value (-1 to 1)
     * @param cropYMax Maximum Y value (-1 to 1)
     */
    public LimelightEx withCropWindow(double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
        LimelightHelpers.setCropWindow(name, cropXMin, cropXMax, cropYMin, cropYMax);
        return this;
    }

    public LimelightEx withIdFilter(int[] ids) {
        LimelightHelpers.SetFiducialIDFiltersOverride(name, ids);
        return this;
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @return a Pose2d object
     */
    public Pose2d getBotPose2d() {
        return LimelightHelpers.getBotPose2d(name);
    }

    public void setPipelineIndex(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(name, pipelineIndex);
    }

    /**
     * Sets robot orientation values used by MegaTag2 localization algorithm.
     * 
     * @param yaw Robot yaw in degrees. 0 = robot facing red alliance wall in FRC
     * @param yawRate (Unnecessary) Angular velocity of robot yaw in degrees per second
     * @param pitch (Unnecessary) Robot pitch in degrees 
     * @param pitchRate (Unnecessary) Angular velocity of robot pitch in degrees per second
     * @param roll (Unnecessary) Robot roll in degrees
     * @param rollRate (Unnecessary) Angular velocity of robot roll in degrees per second
     */
    public void setRobotOrientation(double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
        LimelightHelpers.SetRobotOrientation(name, yaw, yawRate, pitch, pitchRate, roll, rollRate);
    }

    /**
     * Gets the MegaTag2 Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) in the WPILib Blue alliance coordinate system.
     * Make sure you are calling setRobotOrientation() before calling this method.
     * 
     */
    public PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }

    /**
     * Gets the latest raw fiducial/AprilTag detection results from NetworkTables.
     * 
     * @return Array of RawFiducial objects containing detection details
     */
    public RawFiducial[] getRawFiducials() {
        return LimelightHelpers.getRawFiducials(name);
    }

    public String getName() {
        return this.name;
    }

    /**
     * Gets the target area as a percentage of the image (0-100%).
     * 
     * @return Target area percentage (0-100)
     */
    public double getTA() {
        return LimelightHelpers.getTA(name);
    }
    
    /**
     * Gets the horizontal offset from the crosshair to the target in degrees.
     * 
     * @return Horizontal offset angle in degrees
     */
    public double getTX() {
        return LimelightHelpers.getTX(name);
    }
    
    /**
     * Gets the vertical offset from the crosshair to the target in degrees.
     * 
     * @return Vertical offset angle in degrees
     */
    public double getTY() {
        return LimelightHelpers.getTY(name);
    }
    
    /**
     * Does the Limelight have a valid target?
     * 
     * @return True if a valid target is present, false otherwise
     */
    public boolean getTV() {
        return LimelightHelpers.getTV(name);
    }
    
    public double getID() {
        return LimelightHelpers.getFiducialID(name);
    }
 
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("tA", this::getTA, null);
        builder.addDoubleProperty("tX", this::getTX, null);
        builder.addDoubleProperty("tY", this::getTY, null);
        builder.addBooleanProperty("tV", this::getTV, null);
        builder.addDoubleProperty("ID", this::getID, null);  
    }
}
