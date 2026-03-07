package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.DroidRageConstants;
import frc.robot.DroidRageConstants.FieldConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.ClimbValue;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.shooter.Shooter;

public class AutoCommands{

    public AutoCommands() {
        
    }

    public static Command climbDown(Climb climb) {
        return climb.setTargetPositionCommand(ClimbValue.START.getHeight());
    }

    public static Command climbUp(Climb climb) {
        return climb.setTargetPositionCommand(ClimbValue.CLIMB.getHeight());
    }

    public static Command intakeDown(Intake intake) {
        return intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN.getAngle());
    }

    public static Command intakeUp(Intake intake) {
        return intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.UP.getAngle());
    }

    public static Command intake(Intake intake) {
        return intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE);
    }

    public static Command outtake(Intake intake) {
        return intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.OUTTAKE);
    }

    public static Command shootFromBlue(Shooter shooter) {
        return new ParallelCommandGroup(
            shooter.getTurret().setTargetPositionCommand(null),
            shooter.getHood().setTargetPositionCommand(null),
            shooter.getShooterWheel().setTargetVelocityCommand(null)
        );
    }

    public static Command shootFromRed(Shooter shooter) {
        return new ParallelCommandGroup(
            shooter.getTurret().setTargetPositionCommand(null),
            shooter.getHood().setTargetPositionCommand(null),
            shooter.getShooterWheel().setTargetVelocityCommand(null)
        );
    }
}
