package frc.robot.SysID;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModule;

public class DriveSysID {

  // Mutable holders for unit-safe values (shared across all modules)
  private final MutVoltage appliedVoltage = Volts.mutable(0);
  private final MutDistance driveDistance = Inches.mutable(0);
  private final MutLinearVelocity driveVelocity = InchesPerSecond.mutable(0);
  private final MutAngle turnAngle = Radians.mutable(0);
  private final MutAngularVelocity turnVelocity = RadiansPerSecond.mutable(0);

  // SysId Routine for all swerve modules
  private final SysIdRoutine routine;

  /** 
   * Creates a new SysID instance for all swerve modules.
   * @param swerveModules the array of swerve modules to use
   * @param drive the swerve drive subsystem
   */
  public DriveSysID(SwerveModule[] swerveModules, SwerveDrive drive) {
    routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            voltage -> {
              // Apply voltage to all drive and turn motors
              for (SwerveModule module : swerveModules) {
                module.getDriveMotor().setVoltage(voltage);
                module.getTurnMotor().setVoltage(voltage);
              }
            },
            log -> {
              // Log data for all swerve modules
              for (int i = 0; i < swerveModules.length; i++) {
                SwerveModule module = swerveModules[i];

                log.motor("swerve-drive-" + i)
                    .voltage(appliedVoltage.mut_replace(module.getDriveMotor().getVoltage() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(driveDistance.mut_replace(module.getDriveMotor().getPosition(), Inches))
                    .linearVelocity(driveVelocity.mut_replace(module.getDriveMotor().getVelocity(), InchesPerSecond));

                log.motor("swerve-turn-" + i)
                    .voltage(appliedVoltage.mut_replace(module.getTurnMotor().getVoltage() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(turnAngle.mut_replace(module.getTurnMotor().getPosition(), Rotations))
                    .angularVelocity(turnVelocity.mut_replace(module.getTurnMotor().getVelocity(), RotationsPerSecond));
              }
            },
            drive));
  }

  /**
   * @param direction The direction (forward or reverse) to run the test in
   * @return a command that will execute a quasistatic test in the given direction.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  /**
   * @param direction The direction (forward or reverse) to run the test in
   * @return a command that will execute a dynamic test in the given direction.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}