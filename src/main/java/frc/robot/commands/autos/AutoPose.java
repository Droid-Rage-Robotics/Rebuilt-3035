package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public enum AutoPose {
    //Starting Pos
    R_TRENCH_START(new Pose2d(8.25, 3.2, new Rotation2d(90))),
    L_TRENCH_START(new Pose2d(8.25, 3.2, new Rotation2d(90))),
    MID_HUB_START(new Pose2d(8.25, 3.2, new Rotation2d(90))),

    //Intake Ball Pit Pos
    R_OUTPOST_BALL(new Pose2d(8.25, 3.2, new Rotation2d(90))),
    L_DEPOT_BALL(new Pose2d(8.25, 3.2, new Rotation2d(90))),
    R_OUTPOST_BALL_SH(R_OUTPOST_BALL.bluePose.plus(new Transform2d(new Translation2d(-1, 0), new Rotation2d(0)))),
    L_DEPOT_BALL_SH(L_DEPOT_BALL.bluePose.plus(new Transform2d(new Translation2d(-1, 0), new Rotation2d(0)))),


    //Back In Pos
    R_OUTPOST_TRENCH(new Pose2d(8.25, 3.2, new Rotation2d(90))),
    L_DEPOT_TRENCH(new Pose2d(8.25, 3.2, new Rotation2d(90))),

    //Shoot Pos
    R_OUTPOST_SHOOT(new Pose2d(8.25, 3.2, new Rotation2d(90))),
    L_DEPOT_SHOOT(new Pose2d(8.25, 3.2, new Rotation2d(90)))
    ;

    public final Pose2d bluePose;

    AutoPose(Pose2d bluePose) {
        this.bluePose = bluePose;
    }
}
