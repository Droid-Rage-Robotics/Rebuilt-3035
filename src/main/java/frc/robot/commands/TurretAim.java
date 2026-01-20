// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.drive.SwerveDrive;
// import frc.robot.subsystems.shooter.Turret;
// import frc.utility.LimelightEx;

// public class TurretAim extends Command {
//     private final Turret turret;
//     private final PIDController turretController = new PIDController(0, 0, 0);
//     private final SwerveDrive drive;
//     private final LimelightEx turretLimelight;

//     public TurretAim(Turret turret, SwerveDrive drive, LimelightEx turretLimelight) {
//         this.turret=turret;
//         this.drive=drive;
//         this.turretLimelight=turretLimelight;

//         addRequirements(turret);
//     }

//     @Override
//     public void initialize() {
        
//     }

//     @Override
//     public void execute() {
//         var tagError = turretLimelight.getTX();

//         if (drive.getPose().getX()>=4.625||drive.getPose().getX()<=5)  { // if is in alliance zone; dumb values for now
//             turret.setTargetPosition(tagError);
//         }

//         var volts = turretController.calculate(tagError, 0);

//         if (turretLimelight.getTV()) {
//             turret.setVoltage(volts);
//         }

//     }
// }
