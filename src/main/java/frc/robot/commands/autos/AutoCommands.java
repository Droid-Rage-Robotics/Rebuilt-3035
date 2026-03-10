package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;

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
}
