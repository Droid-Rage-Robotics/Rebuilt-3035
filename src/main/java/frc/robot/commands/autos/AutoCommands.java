package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
// import frc.robot.subsystems.Indexer.IndexerValue;
import frc.robot.subsystems.Kicker.KickerValue;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerValue;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterValue;

public class AutoCommands{
    public static Command startPivotCommand(Intake intake){ //TODO: Test
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(()->intake.getPivot().changeCurrentLimit(false)),
                intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN)
            ),
            new WaitCommand(1),
            new InstantCommand(()->intake.getPivot().changeCurrentLimit(true))
        );
    }

    public static Command shootOutpost(Shooter shooter, Indexer indexer, Kicker kicker) {
        return new ParallelCommandGroup(
            shooter.setShooterTargetCommand(ShooterValue.SHOOT_OUTPOST),
            indexer.setTargetVelocityCommand(IndexerValue.INTAKE),
            kicker.setTargetVelocityCommand(KickerValue.INTAKE.getKickerValue())
        );
    }

    public static Command resetBot(Shooter shooter, Indexer indexer, Kicker kicker, Intake intake) {
        return new ParallelCommandGroup(
            shooter.setShooterTargetCommand(ShooterValue.HOLD),
            indexer.setTargetVelocityCommand(Indexer.IndexerValue.STOP),
            kicker.setTargetVelocityCommand(KickerValue.STOP.getKickerValue()),
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN),
            intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP)
        );
    }

    
}
