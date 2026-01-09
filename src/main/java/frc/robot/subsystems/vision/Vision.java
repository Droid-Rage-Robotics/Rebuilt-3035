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
import frc.utility.DashboardUtils;
import frc.utility.DashboardUtils.Dashboard;
import frc.utility.LimelightEx;
import lombok.Getter;

public class Vision extends SubsystemBase implements Dashboard{
    public enum Location {
        // Naming convention is Limelight_Pole_Level_TagID

        RIGHT_R_L4_17(2.14, -19.94), // Default -blue done
        RIGHT_R_L4_18(2.1, -19.8), // done
        RIGHT_R_L4_19(1.87, -20.24), // done 1.87, -20.24
        RIGHT_R_L4_20(1.2, -19.35), // done 1.2, -19.35
        RIGHT_R_L4_21(1.71, -18.83), // done
        RIGHT_R_L4_22(2.01, -19.9), // done

        RIGHT_R_L4_6(2.16, -20.57), // red//done
        RIGHT_R_L4_7(1.84, -19.13), // done
        RIGHT_R_L4_8(1.64, -22.18), // done
        RIGHT_R_L4_9(2.28, -19.06), // done
        RIGHT_R_L4_10(1.6, -16.61), // done
        RIGHT_R_L4_11(1.88, -18.38), // done

        RIGHT_R_LEFT(0, 0),

        // RIGHT_R_L3(RIGHT_R_L4),
        // RIGHT_R_L2(RIGHT_R_L4),

        // ALGAE_R(0,0),

        LEFT_L_L4_17(-0.19, 19.41), // Default -blue //done
        LEFT_L_L4_18(.05, 18.89), // done iffy
        LEFT_L_L4_19(-0.29, 22.12), // dne  -0.29, 22.12
        LEFT_L_L4_20(-0.14, 20.87), // DOne
        LEFT_L_L4_21(-0.25, 18.9), // done
        LEFT_L_L4_22(-0.1, 18.5), // done

        LEFT_L_L4_6(-0.38, 19.7), // red - done - BAD BAD
        LEFT_L_L4_7(-0.06, 19.38), // done
        LEFT_L_L4_8(-0.24, 17.81), // done
        LEFT_L_L4_9(-0.06, 17.67), // done
        LEFT_L_L4_10(-0.3, 19.4), // done
        LEFT_L_L4_11(-0.06, 20.36), // done

        LEFT_L_RIGHT(0,0),

        // LEFT_L_L3(LEFT_L_L4),
        // LEFT_L_L2(LEFT_L_L4),

        // ALGAE_L(0, 0)

        RIGHT_A(1,-3),
        LEFT_A(-.5,4.5),

        ;

        private double distance, angle;

        private Location(double distance, double angle) {
            this.distance = distance;
            this.angle = angle;
        }

        private Location(Location location) {
            this.distance = location.distance;
            this.angle = location.angle;
        }

        public double getDistance() {
            return distance;
        }

        public double getAngle() {
            return angle;
        }
    }

    public enum MountPose {
        R_FORWARD(0.2267388258),
        R_SIDE(0.30880893),
        R_UP(0.2228530468),
        R_ROLL(0),
        R_PITCH(20),
        R_YAW(45),

        L_FORWARD(0.2267388258),
        L_SIDE(-0.30880893),
        L_UP(0.2228530468),
        L_ROLL(0),
        L_PITCH(20),
        L_YAW(-45);

        private final double value;

        private MountPose(double value) {
            this.value=value;
        }

        public double getValue() {
            return value;
        }
    }
    
    public static class Constants {
        public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        public static final double MAX_POSITION_JUMP = 1.0;  // meters
        public static final double MAX_ROTATION_JUMP = 45.0; // degrees
    }

    public int targetIds[];
    public PIDController rotController = new PIDController(.095, 0, 0);// .1
    public PIDController xController = new PIDController(.11, 0, 0);// .1
    private int bluePipeline = 0, redPipeline = 1, leftPipeline =2, leftFrontPipeline=3,rightPipeline =4, rightFrontPipeline=5;
    public final AtomicBoolean isAlign = new AtomicBoolean(false);
    // Set Up the team number - http://limelight.local:5801/

    @Getter private final LimelightEx rightLimelight = LimelightEx.create(DroidRageConstants.rightLimelight) // webgui at 10.30.35.12:5801
        .withStreamMode_Standard()
        .withCropWindow(-1, 1, -1, 1);
    
    @Getter private final LimelightEx leftLimelight = LimelightEx.create(DroidRageConstants.leftLimelight) // webgui at 10.30.35.11:5801
        .withStreamMode_Standard()
        .withCropWindow(-1, 1, -1, 1);

    // Initialize Limelight network tables
    public Vision() {
        // Change the camera pose relative to robot center (x forward, y left, z up,
        // degrees)

        rightLimelight.setMountPose(
            MountPose.R_FORWARD.getValue(), // Forward offset (meters)
            MountPose.R_SIDE.getValue(), // Side offset (meters)
            MountPose.R_UP.getValue(), // Height offset (meters)
            MountPose.R_ROLL.getValue(), // Roll (degrees)
            MountPose.R_PITCH.getValue(), // Pitch (degrees)
            MountPose.R_YAW.getValue() // Yaw (degrees)
        );
        // Change the camera pose relative to robot center (x forward, y left, z up,
        // degrees)
        leftLimelight.setMountPose(
            MountPose.L_FORWARD.getValue(), // Forward offset (meters)
            MountPose.L_SIDE.getValue(), // Side offset (meters)
            MountPose.L_UP.getValue(), // Height offset (meters)
            MountPose.L_ROLL.getValue(), // Roll (degrees)
            MountPose.L_PITCH.getValue(), // Pitch (degrees)
            MountPose.L_YAW.getValue() // Yaw (degrees)
        );

        // for (int port = 5800; port <= 5809; port++) {
        //     PortForwarder.add(port, "limelight.local", port);
        // }

        // setUpVision();
        rotController.setTolerance(.7);//.5
        xController.setTolerance(.7);//.4

        DashboardUtils.registerDashboard(this);
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData("Left Limelight", leftLimelight);
        SmartDashboard.putData("Right Limelight", rightLimelight);
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    @Override
    public void periodic() {}

    public void setUpVision() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            targetIds = new int[] { 6, 7, 8, 9, 10, 11 };
            leftLimelight.setPipelineIndex(redPipeline);
            rightLimelight.setPipelineIndex(redPipeline);

        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
            targetIds = new int[] { 17, 18, 19, 20, 21, 22 };
            leftLimelight.setPipelineIndex(bluePipeline);
            rightLimelight.setPipelineIndex(bluePipeline);
        }
    }

    public void setUpLeftVision(){
        targetIds = new int[] {6,19 };
        leftLimelight.setPipelineIndex(leftPipeline);
        rightLimelight.setPipelineIndex(leftPipeline);
    }
    public void setUpLeftFrontVision(){
        targetIds = new int[] {20,11};
        leftLimelight.setPipelineIndex(leftFrontPipeline);
        rightLimelight.setPipelineIndex(leftFrontPipeline);
    }
    public void setUpRightVision(){
        targetIds = new int[] {8,17};
        leftLimelight.setPipelineIndex(rightPipeline);
        rightLimelight.setPipelineIndex(rightPipeline);
    }
    public void setUpRightFrontVision(){
        targetIds = new int[] {9,22};
        leftLimelight.setPipelineIndex(rightFrontPipeline);
        rightLimelight.setPipelineIndex(rightFrontPipeline);
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    /**
     * Gets the horizontal offset from the crosshair to the target in degrees.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Horizontal offset angle in degrees
     */
    public double getTX(String name) {
        if (name == DroidRageConstants.leftLimelight) {
            return leftLimelight.getTX();
        } else {
            return rightLimelight.getTX();
        }
    }

    /**
     * Gets the target area as a percentage of the image (0-100%).
     * @param limelightName Name of the Limelight camera ("" for default) 
     * @return Target area percentage (0-100)
     */
    public double getTA(String name) {
        if (name == DroidRageConstants.leftLimelight) {
            return leftLimelight.getTA();
        } else {
            return rightLimelight.getTA();
        }
    }

    /**
     * Gets the vertical offset from the crosshair to the target in degrees.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Vertical offset angle in degrees
     */
    public double getTY(String name) {
        if (name == DroidRageConstants.leftLimelight) {
            return leftLimelight.getTY();
        } else {
            return rightLimelight.getTY();
        }
    }

    /**
     * Does the Limelight have a valid target?
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return True if a valid target is present, false otherwise
     */
    public boolean getTV(String name) {
        if (name == DroidRageConstants.leftLimelight) {
            return leftLimelight.getTV();
        } else {
            return rightLimelight.getTV();
        }
    }

    /** The name will be which piepline to use based on which alignment direction */
    // public boolean isID(String name){
    // for (int element : targetIds) {
    // if (element == LimelightHelpers.getFiducialID(name)) {
    // isIDWriter.set(true);
    // return true;
    // }
    // }
    // isIDWriter.set(false);
    // return false;
    // }

    public int getID(String name) {
        if (name == DroidRageConstants.leftLimelight) {
            return (int) leftLimelight.getID();
        } else {
            return (int) rightLimelight.getID();
        }
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
    
    public RawFiducial[] getRawFiducials() {
        return switch (DroidRageConstants.alignmentMode) {
            case LEFT -> leftLimelight.getRawFiducials();
            case RIGHT -> rightLimelight.getRawFiducials();
            case MIDDLE -> {
                if (getTV(DroidRageConstants.leftLimelight)) {
                    yield leftLimelight.getRawFiducials();
                } else if (getTV(DroidRageConstants.rightLimelight)) {
                    yield rightLimelight.getRawFiducials();
                } else {
                    yield null;
                }
            }
        };
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
        return leftLimelight.getBotPoseEstimate_wpiBlue_MegaTag2();
    }

    /**
     * Gets the MegaTag2 Pose2d and timestamp from the right limelight for use with WPILib pose estimator
     * (addVisionMeasurement) in the WPILib Blue alliance coordinate system.
     * Make sure you are calling setRobotOrientation() before calling this method.
     * 
     * @return a new PoseEstimate
     */
    public PoseEstimate getRightEstimate() {
        return rightLimelight.getBotPoseEstimate_wpiBlue_MegaTag2();
    }

    public Location getLeftLocation(String name) {
        int look = getID(name);
        if(DroidRageConstants.alignmentMode==Alignment.MIDDLE){
            return Vision.Location.LEFT_A;
        } else if(DroidRageConstants.alignmentMode == Alignment.RIGHT){
            return Vision.Location.LEFT_L_RIGHT;
        }
        switch (look) {
            case 17:
                return Vision.Location.LEFT_L_L4_17;
            case 18:
                return Vision.Location.LEFT_L_L4_18;
            case 19:
                return Vision.Location.LEFT_L_L4_19;
            case 20:
                return Vision.Location.LEFT_L_L4_20;
            case 21:
                return Vision.Location.LEFT_L_L4_21;
            case 22:
                return Vision.Location.LEFT_L_L4_22;

            case 6:
                return Vision.Location.LEFT_L_L4_6;
            case 7:
                return Vision.Location.LEFT_L_L4_7;
            case 8:
                return Vision.Location.LEFT_L_L4_8;
            case 9:
                return Vision.Location.LEFT_L_L4_9;
            case 10:
                return Vision.Location.LEFT_L_L4_10;
            case 11:
                return Vision.Location.LEFT_L_L4_11;

            default:
                return Vision.Location.LEFT_L_L4_17;
        }

    }

    public Location getRightLocation(String name) {
        int look = getID(name);
        if (DroidRageConstants.alignmentMode == Alignment.MIDDLE) {
            return Vision.Location.RIGHT_A;
        } else if (DroidRageConstants.alignmentMode == Alignment.LEFT) {
            return Vision.Location.RIGHT_R_LEFT;
        }
        switch (look) {
            case 17:
                return Vision.Location.RIGHT_R_L4_17;
            case 18:
                return Vision.Location.RIGHT_R_L4_18;
            case 19:
                return Vision.Location.RIGHT_R_L4_19;
            case 20:
                return Vision.Location.RIGHT_R_L4_20;
            case 21:
                return Vision.Location.RIGHT_R_L4_21;
            case 22:
                return Vision.Location.RIGHT_R_L4_22;

            case 6:
                return Vision.Location.RIGHT_R_L4_6;
            case 7:
                return Vision.Location.RIGHT_R_L4_7;
            case 8:
                return Vision.Location.RIGHT_R_L4_8;
            case 9:
                return Vision.Location.RIGHT_R_L4_9;
            case 10:
                return Vision.Location.RIGHT_R_L4_10;
            case 11:
                return Vision.Location.RIGHT_R_L4_11;
            default:
                return Vision.Location.RIGHT_R_L4_17;
        }
    }

    /**
     * @return angular velocity to align with an april tag
     */
    public double aim() {
        double targetingAngularVelocity = 0;
        switch (DroidRageConstants.alignmentMode) {
            case LEFT:
                targetingAngularVelocity = rotController.calculate(
                        getTX(DroidRageConstants.leftLimelight),
                        getLeftLocation(DroidRageConstants.leftLimelight).getAngle());
                break;
            case RIGHT:
                targetingAngularVelocity = rotController.calculate(
                        getTX(DroidRageConstants.rightLimelight),
                        getRightLocation(DroidRageConstants.rightLimelight).getAngle());
                break;
            case MIDDLE:
                if (getTV(DroidRageConstants.leftLimelight)) {
                    targetingAngularVelocity = rotController.calculate(
                        getTX(DroidRageConstants.leftLimelight),
                        getLeftLocation(DroidRageConstants.leftLimelight).getAngle());
                } else if (getTV(DroidRageConstants.rightLimelight)) {
                    targetingAngularVelocity = rotController.calculate(
                        getTX(DroidRageConstants.rightLimelight),
                        getRightLocation(DroidRageConstants.rightLimelight).getAngle());
                }
        }
        return targetingAngularVelocity;
    }

    /**
     * @return forward speed to reach an april tag
     */
    public double range() {
        double targetingForwardSpeed = 0;
        switch (DroidRageConstants.alignmentMode) {
            case LEFT:
                targetingForwardSpeed = xController.calculate(
                        getTY(DroidRageConstants.leftLimelight),
                        getLeftLocation(DroidRageConstants.leftLimelight).getDistance());
                break;
            case RIGHT:
                targetingForwardSpeed = xController.calculate(
                        getTY(DroidRageConstants.rightLimelight),
                        getRightLocation(DroidRageConstants.rightLimelight).getDistance());
                break;
            case MIDDLE:
                if (getTV(DroidRageConstants.leftLimelight)) {
					targetingForwardSpeed = xController.calculate(
                        getTY(DroidRageConstants.leftLimelight),
                        getLeftLocation(DroidRageConstants.leftLimelight).getDistance());
				} else if (getTV(DroidRageConstants.rightLimelight)) {
                    targetingForwardSpeed = xController.calculate(
                        getTY(DroidRageConstants.rightLimelight),
                        getRightLocation(DroidRageConstants.rightLimelight).getDistance());
                }
                
                break;
        }
        return targetingForwardSpeed;
    }
}