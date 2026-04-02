package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;
import lombok.Getter;

public class Vision extends SubsystemBase {
    public enum MountPose { // ONLY APPLY IN LIMELIGHT WEB UI
        LEFT_FORWARD(Units.inchesToMeters(-10)),
        LEFT_SIDE(Units.inchesToMeters(-10.25)),
        LEFT_UP(Units.inchesToMeters(7.35)),
        LEFT_ROLL(0),
        LEFT_PITCH(55),
        LEFT_YAW(140),
        
        RIGHT_FORWARD(Units.inchesToMeters(-10)),
        RIGHT_SIDE(Units.inchesToMeters(10.25)),
        RIGHT_UP(Units.inchesToMeters(7.35)),
        RIGHT_ROLL(0),
        RIGHT_PITCH(55),
        RIGHT_YAW(-140),
        ;

        // MIDDLE_FORWARD(Units.inchesToMeters(0)),
        // MIDDLE_SIDE(Units.inchesToMeters(0)),
        // MIDDLE_UP(Units.inchesToMeters(0)),
        // MIDDLE_ROLL(0),
        // MIDDLE_PITCH(0),
        // MIDDLE_YAW(0)
        // ;

        @Getter private final double value;

        private MountPose(double value) {
            this.value=value;
        }
    }
    
    public static class Constants {
        public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        
        public static final int[] BLUE_HUB_IDS = {18,19,20,21,24,25,26,27};
        public static final int[] RED_HUB_IDS = {2,3,4,5,8,9,10,11};
    }

    public int targetHubIds[], targetOdoIds[];
    // private int odoPipeline = 0, blueHubPipeline = 1, redHubPipeline = 2;

    public Vision() {
    }

    @Override
    public void periodic() {}

    /**
     * Used to get the distance to the closest april tag seen by the limelight
     * from a pose estimate.
     * 
     * @param est a pose estimate
     * @return distance in meters
     */
    public static double closestTagDistance(PoseEstimate est) {
        if (est.rawFiducials == null || est.rawFiducials.length == 0) return 999;
        double minDist = 999;
        for (var f : est.rawFiducials) {
            minDist = Math.min(minDist, f.distToRobot);
        }
        return minDist;
    }

    /**
     * Used to get the distance to the closest april tag seen by the limelight.
     * 
     * @param rawFiducials an array of raw fiducials from the limelight
     * @return distance in meters
     */
    public static double closestTagDistance(RawFiducial[] rawFiducials) {
        if (rawFiducials == null || rawFiducials.length == 0) return 999;
        double minDist = 999;
        for (var f : rawFiducials) {
            minDist = Math.min(minDist, f.distToRobot);
        }
        return minDist;
    }

    /**
     * Used to get the distance to a specific april tag.
     * 
     * @param rawFiducials an array of raw fiducials from the limelight
     * @param id the id of the april tag
     * @return distance in meters
     */
    public static double getDistanceToTag(RawFiducial[] rawFiducials, int id) {
        if (rawFiducials == null || rawFiducials.length == 0) return 999;
        double distance = 999;
        for (RawFiducial f : rawFiducials) {
            if (f.id == id) {
                distance = Math.min(distance, f.distToRobot);
            }
        }
        return distance;
    }
    
    
    public static double distanceToStdDev(double distMeters) {
        double MIN_STD = 0.05; // 5cm close
        double MAX_STD = 1.0;  // 1m far away
        double SLOPE = 0.15;   // uncertainty per meter
    
        return Math.min(MAX_STD, MIN_STD + SLOPE * distMeters);
    }

    /**
     * @apiNote NOT FULLY FUNCTIONAL YET; WILL ALWAYS RETURN TRUE
     * 
     * Used to prevent large jumps in position or rotation when using
     * limelight odometry by checking differences in position between
     * current and next position estimates.
     * 
     * @param current the curremt position of the robot
     * @param vision the next vision estimate
     * @return true if the position difference is reasonable; false otherwise
     */
    public static boolean isReasonable(Pose2d current, Pose2d vision) {
        double posDiff = current.getTranslation().getDistance(vision.getTranslation());
        double rotDiff = Math.abs(current.getRotation().minus(vision.getRotation()).getDegrees());

        // return posDiff < Constants.MAX_POSITION_JUMP && rotDiff < Constants.MAX_ROTATION_JUMP;
        return true;
    }

    /**
     * Gets the MegaTag2 Pose2d and timestamp from the left limelight for use with WPILib pose estimator
     * (addVisionMeasurement) in the WPILib Blue alliance coordinate system.
     * Make sure you are calling setRobotOrientation() before calling this method.
     * 
     * @return a new PoseEstimate
     */
    public static PoseEstimate getLeftEstimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(DroidRageConstants.leftLL);
    }

    /**
     * Gets the MegaTag2 Pose2d and timestamp from the left limelight for use with WPILib pose estimator
     * (addVisionMeasurement) in the WPILib Blue alliance coordinate system.
     * Make sure you are calling setRobotOrientation() before calling this method.
     * 
     * @return a new PoseEstimate
     */
    public static PoseEstimate getRightEstimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(DroidRageConstants.rightLL);
    }
}