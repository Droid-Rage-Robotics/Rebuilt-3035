package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Kicker.KickerValue;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerValue;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.shooter.Shooter;

public class TeleopCommands {
    /**
     * Returns a command which brings the pivot up and down rapidly after
     * a wait time of 1 second.
     * @param intake 
     * @return a new sequential command group
     */
    public static Command indexerWiggleIntake(Intake intake) {
        return new SequentialCommandGroup(
            new WaitCommand(1),
            new SequentialCommandGroup(
                intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.HALF),
                new WaitCommand(0.5),
                intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN),
                new WaitCommand(0.5)
            ).repeatedly()
        );
        // return new SequentialCommandGroup(
        //     new WaitCommand(0.5),
        //     new SequentialCommandGroup(
        //         intake.getPivot().setTargetPositionCommand(intake.getPivot().getGoalAngle().plus(Degrees.of(0.0000000002))))
        //         .repeatedly()
        //         .raceWith(new WaitCommand(0.1))
        // );
    }

    public static Command operatorRightBumperWhileTrue(Indexer indexer, Kicker kicker, Intake intake) {
        return new SequentialCommandGroup(
            kicker.setTargetVelocityCommand(KickerValue.INTAKE.getKickerValue()),
            new WaitCommand(0.1),
            indexer.setTargetVelocityCommand(IndexerValue.INTAKE),
            new WaitCommand(2),
            intake.getIntakeWheel().setTargetVelocityCommand(RotationsPerSecond.of(-30))
        );
    }

    public static Command operatorPovLeftWhileTrue(Indexer indexer, Kicker kicker, Intake intake, Shooter shooter) {
        return new SequentialCommandGroup(
            new WaitCommand(0.1),
            kicker.setTargetVelocityCommand(KickerValue.INTAKE.getKickerValue()).onlyIf(shooter::isShooterReady),
            indexer.setTargetVelocityCommand(IndexerValue.INTAKE).onlyIf(shooter::isShooterReady),
            new WaitCommand(2),
            intake.getIntakeWheel().setTargetVelocityCommand(RotationsPerSecond.of(-30))
        );
    }

    public static Command operatorRightBumperOnFalse(Indexer indexer, Kicker kicker, Intake intake) {
        return new ParallelCommandGroup(
            // TeleopCommands.lowerHoodCommand(shooter),
            // shooter.setHoodPositionCommand(ShooterValue.HOLD),
            // shooter.getHood().setTargetPositionCommand(Rotation2d.kZero),
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN),
            indexer.setTargetVelocityCommand(IndexerValue.STOP),
            kicker.setTargetVelocityCommand(KickerValue.STOP.getKickerValue())
        );
    }

    // public static Command turboMode(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter) {
    //     return new ParallelCommandGroup(
    //         drive.enableTurboTorque(),
    //         new InstantCommand(()->intake.getPivot().getMotor().changeCurrentLimits(0.5)),
    //         new InstantCommand(()-> intake.getIntakeWheel().getMotor().changeCurrentLimits(0.5)),
    //         new InstantCommand(()-> indexer.getBottomRollers().getMotor().changeCurrentLimits(0.5)),
    //         new InstantCommand(()-> indexer.getTopRoller().getMotor().changeCurrentLimits(0.5)),
    //         new InstantCommand(()-> kicker.getMotor().changeCurrentLimits(0.5))
    //         // new InstantCommand(()-> shooter.getHood().getMotor().changeCurrentLimits(0.5)),
    //         // new InstantCommand(()-> shooter.getTurret().getMotor().changeCurrentLimits(0.5)),
    //     );
    // }
    // public static Command stopTurboMode(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter) {
    //     return new ParallelCommandGroup(
    //         drive.disableTurboTorque(),
    //         new InstantCommand(()->intake.getPivot().getMotor().changeCurrentLimits(1)),
    //         new InstantCommand(()-> intake.getIntakeWheel().getMotor().changeCurrentLimits(1)),
    //         new InstantCommand(()-> indexer.getBottomRollers().getMotor().changeCurrentLimits(1)),
    //         new InstantCommand(()-> indexer.getTopRoller().getMotor().changeCurrentLimits(1)),
    //         new InstantCommand(()-> kicker.getMotor().changeCurrentLimits(1))
    //         // new InstantCommand(()-> shooter.getHood().getMotor().changeCurrentLimits(1)),
    //         // new InstantCommand(()-> shooter.getTurret().getMotor().changeCurrentLimits(1)),
    //     );
    // }
}
