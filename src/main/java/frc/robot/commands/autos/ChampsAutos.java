package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooter.AutoDRShooter;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.intake.Intake.IntakeValue.WheelVelocity;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterValue;
import frc.robot.subsystems.vision.Vision;

public class ChampsAutos {
    public static Command doubleSide(boolean mirror, SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                PathFollow.create("doubleLeftOne")
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

            PathFollow.create("doubleLeftTwo")
                .withVelocity(13.0)
                .withAcceleration(13.0)
                .withResetOdo(false)
                .withMirror(mirror)
                .build(),

            intake.getIntakeWheel().setTargetVelocityCommand(RPM.zero()),

            PathFollow.create("doubleLeftThree")
                .withVelocity(13.0)
                .withAcceleration(13.0)
                .withResetOdo(false)
                .withMirror(mirror)
                .build(),
            AutoCommands.shooterBeReady(mirror, shooter, 
                ShooterValue.AUTO_SHOOT_TRENCH_LEFT_FAR_ONE, ShooterValue.AUTO_SHOOT_TRENCH_RIGHT_FAR_ONE),

            new ParallelCommandGroup(
                // new AutoDRShooter(drive, shooter),

                PathFollow.create("doubleLeftFour")
                    .withVelocity(13.0)
                    .withAcceleration(13.0)
                    .withResetOdo(false)
                    .withMirror(mirror)
                    .build(),

                // new SequentialCommandGroup(
                //     new WaitCommand(1),
                //     AutoCommands.index(indexer, kicker),
                //     new WaitCommand(5)
                // )
                AutoCommands.autoShoot(6, drive, shooter, indexer, kicker)

            
            ),

            AutoCommands.resetBot(shooter, indexer, kicker, intake),
            intake.getIntakeWheel().setTargetVelocityCommand(WheelVelocity.INTAKE),

            new ParallelCommandGroup(
                PathFollow.create("doubleLeftFive")
                .withVelocity(13.0)
                .withAcceleration(13.0)
                .withResetOdo(false)
                .withMirror(mirror)
                .build(),
            AutoCommands.autoShoot(6, drive, shooter, indexer, kicker)

            )
        );
    }
    
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

            new ParallelCommandGroup(
                PathFollow.create("NeutralToTrenchLeft" + depth)
                    .withVelocity(13.0)
                    .withAcceleration(13.0)
                    .withResetOdo(false)
                    .withMirror(mirror)
                    .build(),
                AutoCommands.shooterBeReady(mirror, shooter, 
                    ShooterValue.AUTO_SHOOT_TRENCH_LEFT_FAR_ONE, ShooterValue.AUTO_SHOOT_TRENCH_RIGHT_FAR_ONE)
            ),

            AutoCommands.autoShoot(6, drive, shooter, indexer, kicker),
            // new WaitCommand(3),
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

            new ParallelCommandGroup(
                PathFollow.create("NeutralToTrenchLeftSec")
                    .withVelocity(13.0)
                    .withAcceleration(13.0)
                    .withResetOdo(false)
                    .withMirror(mirror)
                    .build(),
                AutoCommands.shooterBeReady(mirror, shooter, 
                    ShooterValue.AUTO_SHOOT_TRENCH_LEFT_FAR_ONE, ShooterValue.AUTO_SHOOT_TRENCH_RIGHT_FAR_ONE)
            ),
            
            AutoCommands.autoShoot(6, drive, shooter, indexer, kicker)

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
            new ParallelCommandGroup(
                PathFollow.create("NeutralSwoopToTrenchLeft")
                    .withVelocity(13.0)
                    .withAcceleration(13.0)
                    .withResetOdo(false)
                    .withMirror(mirror)
                    .build(),
                AutoCommands.shooterBeReady(mirror, shooter, 
                    ShooterValue.AUTO_SHOOT_TRENCH_LEFT_FAR_ONE, ShooterValue.AUTO_SHOOT_TRENCH_RIGHT_FAR_ONE)
            ),
            AutoCommands.autoShoot(6, drive, shooter, indexer, kicker)
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
                intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.HALF_TWO),
                AutoCommands.shooterBeReady(mirror, shooter, 
                    ShooterValue.AUTO_SHOOT_TRENCH_LEFT_FAR_ONE, ShooterValue.AUTO_SHOOT_TRENCH_RIGHT_FAR_ONE)
            ),
            AutoCommands.autoShoot(6, drive, shooter, indexer, kicker)
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
