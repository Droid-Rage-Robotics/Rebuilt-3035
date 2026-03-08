package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;

public class TeleopCommands {
    public static Command shootIntakeCommand(Intake intake) {
        return new SequentialCommandGroup(
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.UP),
            new WaitCommand(0.5),
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN),
            new WaitCommand(0.5),
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.UP),
            new WaitCommand(0.5),
            intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN)
        );
    }
}
