package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.Turret;
import frc.utility.LimelightEx;

public class TurretAim extends Command {
    private final Turret turret;
    private final SwerveDrive drive;

    public TurretAim(Turret turret, SwerveDrive drive) {
        this.turret=turret;
        this.drive=drive;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (drive.getPose().getX()>=4.625||drive.getPose().getX()<=5)  { // if is in neutral zone; dumb values for now
            turret.setTargetPositionDegrees(0);
        } else if (drive.getPose().getX()>=4.625||drive.getPose().getX()<=5)  { // if is in red alliance zone; stupid values for now
            turret.aimWithLimelight();

        } else if (drive.getPose().getX()>=4.625||drive.getPose().getX()<=5)  { // if is in blue alliance zone; stupid values for now
            turret.aimWithLimelight();
        }
    }
}
