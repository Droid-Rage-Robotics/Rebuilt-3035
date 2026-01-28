package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.TelemetryUpdater;

public class Telemetry implements TelemetryUpdater {
    private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("Drivetrain");
    private final StructPublisher<Rotation2d> yawPublisher;
    private final StructPublisher<Pose2d> posePublisher;
    private final StructPublisher<Pose3d> pose3dPublisher;
    private final StructPublisher<ChassisSpeeds> chassisSpeedsPublisher;


    private final SwerveDrive drive;

    public Telemetry(SwerveDrive drive) {
        this.drive=drive;

        yawPublisher = networkTable.getStructTopic("Yaw", Rotation2d.struct).publish();
        posePublisher = networkTable.getStructTopic("Pose2d", Pose2d.struct).publish();
        pose3dPublisher = networkTable.getStructTopic("Pose3d", Pose3d.struct).publish();
        chassisSpeedsPublisher = networkTable.getStructTopic("ChassisSpeeds", ChassisSpeeds.struct).publish();


        TelemetryUtils.registerTelemetry(this);
    }

    @Override
    public void updateTelemetry() {
        yawPublisher.set(drive.getRotation2d());
        posePublisher.set(drive.getPose());
        pose3dPublisher.set(drive.getPose3d());
    }
}
