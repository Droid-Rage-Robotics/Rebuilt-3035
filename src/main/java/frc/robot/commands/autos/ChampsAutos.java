package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autos.pathplanner.PathFollow;
import frc.robot.commands.shooter.AutoDRShooter;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class ChampsAutos {
    //false = left, true = right
    public static Command sideDepot(String depth, boolean mirror, SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup (
            new ParallelCommandGroup (
                PathFollow.create("TrenchToNeutralLeft" + depth)
                    .withVelocity(11.0)
                    .withAcceleration(11.0)
                    .withResetOdo(true)
                    .withMirror(mirror)
                    .build(),

                new SequentialCommandGroup(
                    AutoCommands.startPivotCommand(intake),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE)
                )
            ),

            PathFollow.create("NeutralToTrenchLeft" + depth)
                .withVelocity(13.0)
                .withAcceleration(13.0)
                .withResetOdo(false)
                .withMirror(mirror)
                .build(),
            new AutoDRShooter(drive, shooter),
            AutoCommands.index(indexer, kicker),
            new WaitCommand(8),
            AutoCommands.resetBot(shooter, indexer, kicker, intake),


            new ParallelCommandGroup (
                PathFollow.create("TrenchToNeutralLeftSec")
                    .withVelocity(13.0)
                    .withAcceleration(13.0)
                    .withResetOdo(false)
                    .withMirror(mirror)
                    .build(),
                new SequentialCommandGroup(
                    AutoCommands.startPivotCommand(intake),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE)
                )
            ),

            PathFollow.create("NeutralToTrenchLeftSec")
                .withVelocity(13.0)
                .withAcceleration(13.0)
                .withResetOdo(false)
                .withMirror(mirror)
                .build(),
            new AutoDRShooter(drive, shooter),
            AutoCommands.index(indexer, kicker)
        );
    }

    public static Command neutralSwoop(boolean mirror, SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                PathFollow.create("TrenchToNeutralSwoopLeft")
                    .withVelocity(13.0)
                    .withAcceleration(13.0)
                    .withResetOdo(true)
                    .withMirror(mirror)
                    .build(),
                new SequentialCommandGroup(
                    AutoCommands.startPivotCommand(intake),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE)
                )
            ),
            PathFollow.create("NeutralSwoopToTrenchLeft")
                .withVelocity(13.0)
                .withAcceleration(13.0)
                .withResetOdo(false)
                .withMirror(mirror)
                .build(),
            new AutoDRShooter(drive, shooter),
            AutoCommands.index(indexer, kicker)
        );
    }

    public static Command bump(boolean mirror, SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                PathFollow.create("Bump1")
                    .withVelocity(13.0)
                    .withAcceleration(13.0)
                    .withResetOdo(true)
                    .withMirror(mirror)
                    .build(),
                new SequentialCommandGroup(
                    new WaitCommand(1.5),
                    AutoCommands.startPivotCommand(intake),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE)
                )
            ),
            new WaitCommand(.5),
            new ParallelCommandGroup(
                PathFollow.create("Bump2")
                .withVelocity(13.0)
                .withAcceleration(13.0)
                .withResetOdo(false)
                .withMirror(mirror)
                .build(),
                intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP),
                intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.HALF_TWO)
            ),
            new AutoDRShooter(drive, shooter),
            AutoCommands.index(indexer, kicker)
        );
    }

    public static Command centerDepot(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                PathFollow.create("HubToDepot")
                    .withVelocity(13.0)
                    .withAcceleration(13.0)
                    .withResetOdo(true)
                    .withMirror(false)
                    .build(),
                new SequentialCommandGroup(
                    intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.AUTO_DOWN),
                    new WaitCommand(0.25),
                    intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE)
                )
            ),
            new WaitCommand(.5),
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN),
            new WaitCommand(.5),

            new ParallelCommandGroup(
                PathFollow.create("HubToDepot2")
                    .withVelocity(13.0)
                    .withAcceleration(13.0)
                    .withResetOdo(false)
                    .withMirror(false)
                    .build(),
                new SequentialCommandGroup(
                    new WaitCommand(1.5),
                    shooter.setShooterTargetCommand(Shooter.ShooterValue.AUTO_DEPOT),
                    new WaitCommand(.4),
                    AutoCommands.index(indexer, kicker),
                    new WaitCommand(.5),
                    new ParallelCommandGroup(
                        AutoCommands.autoWiggleIntake(intake)
                    )
                )
            ),
            // new WaitCommand(8), //Remove if wiggling indexer
            new ParallelCommandGroup(
                shooter.setShooterTargetCommand(Shooter.ShooterValue.HOLD),
                PathFollow.create("HubToDepot3")
                    .withVelocity(13.0)
                    .withAcceleration(13.0)
                    .withResetOdo(false)
                    .withMirror(false)
                    .build()
                
            )
        );
    }
}
