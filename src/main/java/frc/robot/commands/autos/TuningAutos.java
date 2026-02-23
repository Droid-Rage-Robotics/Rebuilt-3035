package frc.robot.commands.autos;

import frc.robot.subsystems.drive.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class TuningAutos {
    
    public static Command forwardTest(SwerveDrive drive) {//Top Red/Bottom Blue
        // PathPlannerPath path = PathPlannerPath.fromPathFile("ForwardTest");
        return new SequentialCommandGroup(
            // Commands.runOnce(() -> drive.resetOdometry(path.getPreviewStartingHolonomicPose())),
            // PathPlannerFollow.
            // drive.
            PathPlannerFollow.create(drive, "ForwardTest")
            .setMaxVelocity(1)
            .setAcceleration(1)
                .build()
        );
    }
    public static Command backTest(SwerveDrive drive) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "BackwardTest")
                .setMaxVelocity(1)
                .setAcceleration(1)
                .build()
        );
    }
    public static Command turnTest(SwerveDrive drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "TurnTest")
                .setMaxVelocity(2)
                .setAcceleration(2)
                .build()
            // drive.setYawCommand(drive.getRotation2d().rotateBy(Rotation2d.fromDegrees(180)).getDegrees())//Works
        );

    }
    public static Command splineTest(SwerveDrive drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "SplineTest")
                .setMaxVelocity(1)
                .setAcceleration(1)
                .build()
        );
    }
    
    public static Command strafeRight(SwerveDrive drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "StrafeRightTest")
                .setMaxVelocity(0.2)
                .setAcceleration(0.2)
                .build()
        );
    }
    public static Command strafeLeft(SwerveDrive drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "StrafeLeftTest")
                .setMaxVelocity(0.2)
                .build()
        );
    }

    public static Command lessForwardTest(SwerveDrive drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "LessForwardTest")
                .setMaxVelocity(0.2)
                .build()
        );
    }
    
    private TuningAutos() {}
}
