package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooter.DRShooter;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Kicker.KickerValue;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerValue;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterValue;

public class AutoCommands{
    public static Command startPivotCommand(Intake intake){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                // new InstantCommand(()->intake.getPivot().turnCurrentLimitOff()),
                intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN)
            ),
            new WaitCommand(1)
            // ,
            // new InstantCommand(()->intake.getPivot().turnCurrentLimitOn())
        );
    }

    public static Command shootOutpost(Shooter shooter, Indexer indexer, Kicker kicker) {
        return new ParallelCommandGroup(
            shooter.setShooterTargetCommand(ShooterValue.SHOOT_OUTPOST),
            indexer.setTargetVelocityCommand(IndexerValue.INTAKE),
            kicker.setTargetVelocityCommand(KickerValue.INTAKE.getKickerValue())
        );
    }

    public static Command autoShoot(int raceLength, SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Intake intake) {
        return new ParallelCommandGroup (
            new DRShooter(drive, shooter),
            new SequentialCommandGroup(
                new WaitCommand(.5),
            AutoCommands.index(indexer, kicker),
            AutoCommands.autoWiggleIntake(intake)

            )
        ).raceWith(
            new WaitCommand(raceLength)
        );
    }
    public static Command autoShootWithIntake(int raceLength, SwerveDrive drive, Shooter shooter, Indexer indexer, Kicker kicker, Intake intake) {
        return new ParallelCommandGroup (
            new DRShooter(drive, shooter),
            new SequentialCommandGroup(
                new WaitCommand(.5),
            AutoCommands.index(indexer, kicker),
            intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE)
            )
        ).raceWith(
            new WaitCommand(raceLength)
        );
    }

    public static Command shooterBeReady(boolean mirror, Shooter shooter, ShooterValue left, ShooterValue right) {
        return new ConditionalCommand(
            shooter.setTurretCommand(right), // onTrue
            shooter.setTurretCommand(left), // onFalse
            () -> mirror
        );
    }

    public static Command resetBot(Shooter shooter, Indexer indexer, Kicker kicker, Intake intake) {
        return new ParallelCommandGroup(
            // shooter.setShooterTargetCommand(ShooterValue.HOLD),
            shooter.getHood().setTargetPositionCommand(Degrees.zero()),
            shooter.getShooterWheel().setTargetVelocityCommand(Shooter.IDLE_VELOCITY),
            indexer.setTargetVelocityCommand(Indexer.IndexerValue.STOP),
            kicker.setTargetVelocityCommand(KickerValue.STOP.getKickerValue()),
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN),
            intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP)
        );
    }

    public static Command autoWiggleIntake(Intake intake) {
        return new SequentialCommandGroup(
            new WaitCommand(1),
            new SequentialCommandGroup(
                intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.HALF),
                new WaitCommand(0.5),
                intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN),
                new WaitCommand(0.5)
            ).repeatedly().raceWith(new WaitCommand(8)) // Wait to START A New Cycle
        );
    }

    public static Command index(Indexer indexer, Kicker kicker) {
        return new SequentialCommandGroup(
            indexer.setTargetVelocityCommand(IndexerValue.INTAKE),
            kicker.setTargetVelocityCommand(KickerValue.INTAKE.getKickerValue())
        );
    }

    
}
