package frc.robot.commands.SysId;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SysID.SysID;

public class SysIdCommand extends SequentialCommandGroup {
    public SysIdCommand(SysID sysId) {
        addCommands(
            sysId.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
            new WaitCommand(1),
            sysId.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
            new WaitCommand(1),
            sysId.sysIdDynamic(SysIdRoutine.Direction.kForward),
            new WaitCommand(1),
            sysId.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );
    }
}