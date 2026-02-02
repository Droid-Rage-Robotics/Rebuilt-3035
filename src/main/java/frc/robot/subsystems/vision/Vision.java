package frc.robot.subsystems.vision;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants;
import frc.robot.DroidRageConstants.Alignment;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.Dashboard;
import frc.utility.LimelightEx;
import lombok.Getter;

public class Vision extends SubsystemBase implements Dashboard{
    public enum MountPose {
        ODO_FORWARD(0.2267388258),
        ODO_SIDE(0.30880893),
        ODO_UP(0.2228530468),
        ODO_ROLL(0),
        ODO_PITCH(20),
        ODO_YAW(45),
        
        TURRET_FORWARD(0.2267388258),
        TURRET_SIDE(0.30880893),
        TURRET_UP(0.2228530468),
        TURRET_ROLL(0),
        TURRET_PITCH(20),
        TURRET_YAW(45);

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

    @Getter private final LimelightEx odoLL = LimelightEx.create(DroidRageConstants.odoLL) // webgui at 10.30.35.12:5801
        .withStreamMode_Standard()
        .withFieldLayout(Constants.FIELD_LAYOUT)
        .withCropWindow(-1, 1, -1, 1);
    @Getter private final LimelightEx turretLL = LimelightEx.create(DroidRageConstants.turretLL) // webgui at 10.30.35.12:5801
        .withStreamMode_Standard()
        .withFieldLayout(Constants.FIELD_LAYOUT)
        .withCropWindow(-1, 1, -1, 1);


    // Initialize Limelight network tables
    public Vision() {
        // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
        odoLL.setMountPose(
            MountPose.ODO_FORWARD.getValue(), // Forward offset (meters)
            MountPose.ODO_SIDE.getValue(), // Side offset (meters)
            MountPose.ODO_UP.getValue(), // Height offset (meters)
            MountPose.ODO_ROLL.getValue(), // Roll (degrees)
            MountPose.ODO_PITCH.getValue(), // Pitch (degrees)
            MountPose.ODO_YAW.getValue() // Yaw (degrees)
        );
        turretLL.setMountPose(
            MountPose.TURRET_FORWARD.getValue(), // Forward offset (meters) - Will Change; Should not cause issues
            MountPose.TURRET_SIDE.getValue(), // Side offset (meters) - Will Change; Should not cause issues
            MountPose.TURRET_UP.getValue(), // Height offset (meters) - Will Change; Should not cause issues
            MountPose.TURRET_ROLL.getValue(), // Roll (degrees)
            MountPose.TURRET_PITCH.getValue(), // Pitch (degrees)
            MountPose.TURRET_YAW.getValue() // Yaw (degrees) - Will Change; Should not cause issues
        );

        TelemetryUtils.registerDashboard(this);
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData("ODO_LL", odoLL);
        SmartDashboard.putData("Turret_LL", turretLL);

        
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
            odoLL.setPipelineIndex(odoPipeline);
            turretLL.setPipelineIndex(redHubPipeline);

        } else if (DroidRageConstants.alliance == Alliance.Blue) {
            targetHubIds = new int[] { 18,19,20,21,24,25,26,27};
            targetOdoIds = new int[] {};// TODO:Set Up

            odoLL.setPipelineIndex(odoPipeline);
            turretLL.setPipelineIndex(blueHubPipeline);
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
    public PoseEstimate getOdoEstimate() {
        return odoLL.getBotPoseEstimate_wpiBlue_MegaTag2();
    }
}