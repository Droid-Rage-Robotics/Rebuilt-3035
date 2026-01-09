package frc.robot.commands.SysId;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ManualSysIdRoutine {
    public ManualSysIdRoutine(CommandXboxController driver, SysIdRoutine routine) {
        driver.y().onTrue(routine.dynamic(SysIdRoutine.Direction.kForward))
            .onFalse(new InstantCommand(() -> routine.dynamic(SysIdRoutine.Direction.kForward).cancel()));

        driver.a().onTrue(routine.dynamic(SysIdRoutine.Direction.kReverse))
            .onFalse(new InstantCommand(() -> routine.dynamic(SysIdRoutine.Direction.kReverse).cancel()));

        driver.x().onTrue(routine.quasistatic(SysIdRoutine.Direction.kReverse))
            .onFalse(new InstantCommand(() -> routine.quasistatic(SysIdRoutine.Direction.kReverse).cancel()));

        driver.b().onTrue(routine.quasistatic(SysIdRoutine.Direction.kForward))
            .onFalse(new InstantCommand(() -> routine.quasistatic(SysIdRoutine.Direction.kForward).cancel()));
    }
}

