package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Turret;
import frc.utility.LimelightEx;

public class TurretAim extends Command {
    private final Turret turret;
    private final LimelightEx turretLimelight;

    public TurretAim(Turret turret, LimelightEx turretLimelight) {
        this.turret = turret;
        this.turretLimelight = turretLimelight;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        var yawGoal = turretLimelight.getTX();

        turret.setTargetPosition(yawGoal);
    }
}
