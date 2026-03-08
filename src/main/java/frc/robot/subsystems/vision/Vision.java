package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.Dashboard;
import frc.utility.devices.LimelightEx;
import lombok.Getter;

public class Vision extends SubsystemBase implements Dashboard{
    public enum MountPose {
        LEFT_FORWARD(Units.inchesToMeters(-10)),
        LEFT_SIDE(Units.inchesToMeters(-10.25)),
        LEFT_UP(Units.inchesToMeters(7.35)),
        LEFT_ROLL(0),
        LEFT_PITCH(55),
        LEFT_YAW(-40),
        
        RIGHT_FORWARD(Units.inchesToMeters(-10)),
        RIGHT_SIDE(Units.inchesToMeters(10.25)),
        RIGHT_UP(Units.inchesToMeters(7.35)),
        RIGHT_ROLL(0),
        RIGHT_PITCH(55),
        RIGHT_YAW(40),

        MIDDLE_FORWARD(Units.inchesToMeters(0)),
        MIDDLE_SIDE(Units.inchesToMeters(0)),
        MIDDLE_UP(Units.inchesToMeters(0)),
        MIDDLE_ROLL(0),
        MIDDLE_PITCH(0),
        MIDDLE_YAW(0)
        ;

        @Getter private final double value;

        private MountPose(double value) {
            this.value=value;
        }
    }
    
    public static class Constants {
        public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    }

    public int targetHubIds[], targetOdoIds[];
    private int odoPipeline = 0, blueHubPipeline = 1, redHubPipeline = 2;
    // Set Up the team number - http://limelight.local:5801/

    @Getter private final LimelightEx leftLL = LimelightEx.create(DroidRageConstants.leftLL) // webgui at 10.30.35.12:5801
        .withStreamMode_Standard()
        .withFieldLayout(Constants.FIELD_LAYOUT)
        .withCropWindow(-1, 1, -1, 1);
    @Getter private final LimelightEx rightLL = LimelightEx.create(DroidRageConstants.rightLL) // webgui at 10.30.35.12:5801
        .withStreamMode_Standard()
        .withFieldLayout(Constants.FIELD_LAYOUT)
        .withCropWindow(-1, 1, -1, 1);

    @Getter private final LimelightEx middleLL = LimelightEx.create(DroidRageConstants.middleLL)
        .withStreamMode_Standard()
        .withFieldLayout(Constants.FIELD_LAYOUT)
        .withCropWindow(-1, 1, -1, 1);


    // Initialize Limelight network tables
    public Vision() {
        // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
        leftLL.setMountPose(
            MountPose.LEFT_FORWARD.getValue(), // Forward offset (meters)
            MountPose.LEFT_SIDE.getValue(), // Side offset (meters)
            MountPose.LEFT_UP.getValue(), // Height offset (meters)
            MountPose.LEFT_ROLL.getValue(), // Roll (degrees)
            MountPose.LEFT_PITCH.getValue(), // Pitch (degrees)
            MountPose.LEFT_YAW.getValue() // Yaw (degrees)
        );
        rightLL.setMountPose(
            MountPose.RIGHT_FORWARD.getValue(), // Forward offset (meters) - Will Change; Should not cause issues
            MountPose.RIGHT_SIDE.getValue(), // Side offset (meters) - Will Change; Should not cause issues
            MountPose.RIGHT_UP.getValue(), // Height offset (meters) - Will Change; Should not cause issues
            MountPose.RIGHT_ROLL.getValue(), // Roll (degrees)
            MountPose.RIGHT_PITCH.getValue(), // Pitch (degrees)
            MountPose.RIGHT_YAW.getValue() // Yaw (degrees) - Will Change; Should not cause issues
        );

        TelemetryUtils.registerDashboard(this);
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData("Left_LL", leftLL);
        SmartDashboard.putData("Right_LL", rightLL);

        
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    @Override
    public void periodic() {}

    public void setUpVision() {
        if (DroidRageConstants.alliance == Alliance.Red) {
            targetHubIds = new int[] { 2,3,4,5,8,9,10,11};
            targetOdoIds= new int [] {}; //TODO:Set Up
            leftLL.setPipelineIndex(odoPipeline);
            rightLL.setPipelineIndex(redHubPipeline);

        } else if (DroidRageConstants.alliance == Alliance.Blue) {
            targetHubIds = new int[] { 18,19,20,21,24,25,26,27};
            targetOdoIds = new int[] {};// TODO:Set Up

            leftLL.setPipelineIndex(odoPipeline);
            rightLL.setPipelineIndex(blueHubPipeline);
        }
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }
    
    /**
     * Used to get the distance to the closest april tag seen by the limelight
     * from a pose estimate.
     * 
     * @param est a pose estimate
     * @return distance in meters
     */
    public double closestTagDistance(PoseEstimate est) {
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
    public double closestTagDistance(RawFiducial[] rawFiducials) {
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
    public double getDistanceToTag(RawFiducial[] rawFiducials, int id) {
        if (rawFiducials == null || rawFiducials.length == 0) return 999;
        double distance = 999;
        for (RawFiducial f : rawFiducials) {
            if (f.id == id) {
                distance = Math.min(distance, f.distToRobot);
            }
        }
        return distance;
    }
    
    
    public double distanceToStdDev(double distMeters) {
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
    public boolean isReasonable(Pose2d current, Pose2d vision) {
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
    public PoseEstimate getLeftEstimate() {
        return leftLL.getBotPoseEstimate_wpiBlue_MegaTag2();
    }

    /**
     * Gets the MegaTag2 Pose2d and timestamp from the left limelight for use with WPILib pose estimator
     * (addVisionMeasurement) in the WPILib Blue alliance coordinate system.
     * Make sure you are calling setRobotOrientation() before calling this method.
     * 
     * @return a new PoseEstimate
     */
    public PoseEstimate getRightEstimate() {
        return rightLL.getBotPoseEstimate_wpiBlue_MegaTag2();
    }
}