package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public final class Autos {
    public static Command rightNeutralOutpost(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "test") //"rightNeutralOutpost"
                .setMaxVelocity(8)
                .setAcceleration(8)
                .build()
        );
    }

    public static Command rightNeutralOutpostDouble(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "rightNeutralOutpost")
                .setMaxVelocity(8)
                .setAcceleration(8)
                .build(),
            // shooter.setShooterTargetCommand(ShooterValue.AUTO_SHOOT_TRENCH_RIGHT),
            PathPlannerFollow.create(drive, "rightNeutralOutpost2")
                .setMaxVelocity(8)
                .setAcceleration(8)
                .build()
        );
    }

    public static Command leftNeutralDepot(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "leftNeutralDepot")
                .setMaxVelocity(8)
                .setAcceleration(8)
                .build()
        );
    }

    public static Command leftNeutralDepotDouble(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "leftNeutralDepot")
                .setMaxVelocity(8)
                .setAcceleration(8)
                .build(),
            PathPlannerFollow.create(drive, "leftNeutralDepot2")
                .setMaxVelocity(8)
                .setAcceleration(8)
                .build()
        );
    }
}