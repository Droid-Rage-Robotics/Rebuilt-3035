package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Indexer.IndexerValue;
import frc.robot.subsystems.Kicker.KickerValue;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.shooter.Shooter;

public class TeleopCommands {
    public static Command shootIntakeCommand(Intake intake) {
        return new SequentialCommandGroup(
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.HALF_THREE),
            new WaitCommand(1),
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN),
            new WaitCommand(1),
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.HALF_TWO),
            new WaitCommand(1),
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN),
            new WaitCommand(1),
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.HALF_ONE),
            new WaitCommand(1),
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN),
            new WaitCommand(1)
        );
    }

    public static Command indexerIntakeCommand(Indexer indexer) {
        return new SequentialCommandGroup(
            indexer.setTargetVelocityCommand(Indexer.IndexerValue.INTAKE.getIndexerValue()),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    indexer.setTargetVelocityCommand(Indexer.IndexerValue.OUTTAKE.getIndexerValue()),
                    new WaitCommand(.5),
                    indexer.setTargetVelocityCommand(Indexer.IndexerValue.INTAKE.getIndexerValue())
                ),
                new SequentialCommandGroup(),
                () ->  indexer.isStalling()
            )
        );
    }
    
    public static Command indexerWiggleIntake(Intake intake) {
        return new SequentialCommandGroup(
            new WaitCommand(3),
            new SequentialCommandGroup(
                intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.HALF),
                intake.getIntakeWheel().setTargetVelocityCommand(RotationsPerSecond.of(50)),
                new WaitCommand(0.5),
                intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN),
                intake.getIntakeWheel().setTargetVelocityCommand(RotationsPerSecond.of(-50)),
                new WaitCommand(0.5)
            ).repeatedly()
        );
    }

    public static Command operatorRightBumperOnTrue(Shooter shooter, Indexer indexer, Kicker kicker) {
        return new SequentialCommandGroup(
            TeleopCommands.raiseHoodCommand(shooter),
            new WaitCommand(0.2),
            kicker.setTargetVelocityCommand(KickerValue.INTAKE.getKickerValue()),
            new WaitCommand(0.1),
            indexer.setTargetVelocityCommand(IndexerValue.INTAKE.getIndexerValue())
        );
    }

    public static Command operatorRightBumperOnFalse(Shooter shooter, Indexer indexer, Kicker kicker) {
        return new ParallelCommandGroup(
            TeleopCommands.lowerHoodCommand(shooter),
            indexer.setTargetVelocityCommand(IndexerValue.STOP.getIndexerValue()),
            kicker.setTargetVelocityCommand(KickerValue.STOP.getKickerValue())
        );
    }

    public static Command raiseHoodCommand(Shooter shooter) {
        return shooter.getHood().setTargetPositionCommand(shooter.getShooterValue().getHoodAngle());
    }

    public static Command lowerHoodCommand(Shooter shooter) {
        return shooter.getHood().setTargetPositionCommand(Rotation2d.kZero);
    }
}
