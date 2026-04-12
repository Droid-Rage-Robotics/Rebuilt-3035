package frc.robot.commands.autos;

import frc.robot.subsystems.drive.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class TuningAutos {
    public static Command forwardTest(SwerveDrive drive) {
        return new SequentialCommandGroup(
            PathPlannerPathFollow.create(drive, "ForwardTest")
                .withMaxVelocity(1)
                .withMaxAcceleration(1)
                .build()
        );
    }
    public static Command backTest(SwerveDrive drive) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerPathFollow.create(drive, "BackwardTest")
                .withMaxVelocity(1)
                .withMaxAcceleration(1)
                .build()
        );
    }
    public static Command turnTest(SwerveDrive drive) {
        return new SequentialCommandGroup(
            PathPlannerPathFollow.create(drive, "TurnTest")
                .withMaxVelocity(2)
                .withMaxAcceleration(2)
                .build()
        );

    }
    public static Command splineTest(SwerveDrive drive) {
        return new SequentialCommandGroup(
            PathPlannerPathFollow.create(drive, "SplineTest")
                .withMaxVelocity(1)
                .withMaxAcceleration(1)
                .build()
        );
    }
    
    public static Command strafeRight(SwerveDrive drive) {
        return new SequentialCommandGroup(
            PathPlannerPathFollow.create(drive, "StrafeRightTest")
                .withMaxVelocity(0.2)
                .withMaxAcceleration(0.2)
                .build()
        );
    }
    public static Command strafeLeft(SwerveDrive drive) {
        return new SequentialCommandGroup(
            PathPlannerPathFollow.create(drive, "StrafeLeftTest")
                .withMaxVelocity(0.2)
                .build()
        );
    }

    public static Command lessForwardTest(SwerveDrive drive) {
        return new SequentialCommandGroup(
            PathPlannerPathFollow.create(drive, "LessForwardTest")
                .withMaxVelocity(0.2)
                .build()
        );
    }
    
    private TuningAutos() {}
}
