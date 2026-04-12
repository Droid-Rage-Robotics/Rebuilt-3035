package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.DroidRageConstants;
import frc.robot.commands.shooter.AutoDRShooter;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Kicker.KickerValue;
import frc.robot.subsystems.drive.SwerveConfig;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerValue;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterValue;
import frc.robot.subsystems.vision.Vision;

public final class Autos {
    // public static Command rightNeutralOutpost(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "rightNeutralOutpost")
    //         // PathPlannerFollow.create(drive, "test")
    //             .setMaxVelocity(8)
    //             .setAcceleration(8)
    //             .build()
    //     );
    // }
    // public static Command rightNeutralOutpostSh(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "rightNeutralOutpostSh")
    //         // PathPlannerFollow.create(drive, "test")
    //             .setMaxVelocity(8)
    //             .setAcceleration(8)
    //             .build()
    //     );
    // }

    // public static Command centerHubDepot(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "centerHubDepot")
    //             .setMaxVelocity(8)
    //             .setAcceleration(8)
    //             .build()
    //     );
    // }

    // public static Command leftNeutralDepot(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
    //     return new SequentialCommandGroup(
    //         PathPlannerFollow.create(drive, "leftNeutralDepot")
    //             .setMaxVelocity(8)
    //             .setAcceleration(8)
    //             .build()
    //     );
    // }
    
    public static Command leftNeutralDepotSh(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "leftNeutralDepotSh")
                .setMaxVelocity(8)
                .setAcceleration(8)
                .build()
        );
    }

    public static Command newRightOutpost(String depth, SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup (
            new ParallelCommandGroup (
                PathPlannerPathFollow.create(drive, "TrenchToNeutralRight" + depth, true)
                    .withMaxVelocity(13)
                    .withMaxAcceleration(13)
                    .build(),

                new SequentialCommandGroup(
                    AutoCommands.startPivotCommand(intake),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE) 
                )
            ),

            PathPlannerPathFollow.create(drive, "NeutralToTrenchRight" + depth)
                .withMaxVelocity(13)
                .withMaxAcceleration(13)
                .build(),

            new AutoDRShooter(drive, shooter)
                .repeatedly()
                .alongWith(AutoCommands.autoIndexerWiggleIntake(intake), AutoCommands.index(indexer, kicker))
                .raceWith(new WaitCommand(5)),

            AutoCommands.resetBot(shooter, indexer, kicker, intake),

            new ParallelCommandGroup (
                PathPlannerPathFollow.create(drive, "TrenchToNeutralRightSec")
                    .withMaxVelocity(13)
                    .withMaxAcceleration(13)
                    .build(),

                new SequentialCommandGroup(
                    AutoCommands.startPivotCommand(intake),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE)
                )
            ),

            PathPlannerPathFollow.create(drive, "NeutralToTrenchRightSec")
                .withMaxVelocity(13)
                .withMaxAcceleration(13)
                .build(),

            new AutoDRShooter(drive, shooter)
                .repeatedly()
                .alongWith(AutoCommands.autoIndexerWiggleIntake(intake), AutoCommands.index(indexer, kicker))
        );
    }

    public static Command newLeftDepot(String depth, SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup (
            new ParallelCommandGroup (
                PathPlannerPathFollow.create(drive, "TrenchToNeutralLeft" + depth, true)
                    .withMaxVelocity(13)
                    .withMaxAcceleration(13)
                    .build(),

                new SequentialCommandGroup(
                    AutoCommands.startPivotCommand(intake),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE)
                )
            ),

            PathPlannerPathFollow.create(drive, "NeutralToTrenchLeft" + depth)
                .withMaxVelocity(13)
                .withMaxAcceleration(13)
                .build(),

            new AutoDRShooter(drive, shooter)
                .repeatedly()
                .alongWith(AutoCommands.autoIndexerWiggleIntake(intake), AutoCommands.index(indexer, kicker))
                .raceWith(new WaitCommand(5)),

            AutoCommands.resetBot(shooter, indexer, kicker, intake),

            new ParallelCommandGroup (
                PathPlannerPathFollow.create(drive, "TrenchToNeutralLeftSec")
                    .withMaxVelocity(13)
                    .withMaxAcceleration(13)
                    .build(),

                new SequentialCommandGroup(
                    AutoCommands.startPivotCommand(intake),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE)
                )
            ),

            PathPlannerPathFollow.create(drive, "NeutralToTrenchLeftSec")
                .withMaxVelocity(13)
                .withMaxAcceleration(13)
                .build(),

            new AutoDRShooter(drive, shooter)
                .repeatedly()
                .alongWith(AutoCommands.autoIndexerWiggleIntake(intake), AutoCommands.index(indexer, kicker))
        );
    }
}