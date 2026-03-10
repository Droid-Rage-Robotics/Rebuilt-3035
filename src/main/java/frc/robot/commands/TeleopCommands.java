package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;

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
}
