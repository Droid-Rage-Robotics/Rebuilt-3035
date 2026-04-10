package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum AutoPose {
    R_OUTPOST_BALL(new Pose2d(8.25, 3.2, new Rotation2d(90))),
    L_DEPOT_BALL(new Pose2d(8.25, 3.2, new Rotation2d(90))),
    R_OUTPOST_TRENCH(new Pose2d(8.25, 3.2, new Rotation2d(90))),
    L_DEPOT_TRENCH(new Pose2d(8.25, 3.2, new Rotation2d(90)))
    ;

    public final Pose2d bluePose;

    AutoPose(Pose2d bluePose) {
        this.bluePose = bluePose;
        // this.redPose = redPose;
    }
}
