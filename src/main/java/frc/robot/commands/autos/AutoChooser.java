package frc.robot.commands.autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TeleopCommands;
// import frc.robot.subsystems.Indexer.IndexerValue;
import frc.robot.subsystems.Kicker.KickerValue;
// import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerValue;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.intake.Intake.IntakeValue.WheelVelocity;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterValue;
import frc.robot.subsystems.vision.Vision;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.Dashboard;

public class AutoChooser implements Dashboard {
    public static final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public AutoChooser(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision){
        NamedCommands.registerCommand("startPivot", AutoCommands.startPivotCommand(intake));
        
        NamedCommands.registerCommand("intakeDown",  intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN.getAngle()));
        NamedCommands.registerCommand("intakeUp", intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.UP.getAngle()));

        NamedCommands.registerCommand("intake", intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE));
        NamedCommands.registerCommand("outtake", intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.OUTTAKE));

        NamedCommands.registerCommand("shootTrenchR", shooter.setShooterTargetCommand(ShooterValue.SHOOT_TRENCH_RIGHT));
        NamedCommands.registerCommand("shootTrenchL", shooter.setShooterTargetCommand(ShooterValue.SHOOT_TRENCH_LEFT));

        NamedCommands.registerCommand("setShootTrenchR", shooter.setShooterTargetCommand(ShooterValue.AUTO_SHOOT_TRENCH_RIGHT));
        NamedCommands.registerCommand("setShootTrenchL", shooter.setShooterTargetCommand(ShooterValue.AUTO_SHOOT_TRENCH_LEFT));

        NamedCommands.registerCommand("intakeWait", intake.setTargetVelocityWaitCommand(WheelVelocity.INTAKE));

        NamedCommands.registerCommand("shootOutpost", AutoCommands.shootOutpost(shooter, indexer, kicker));
        NamedCommands.registerCommand("resetBot", AutoCommands.resetBot(shooter, indexer, kicker, intake));

        NamedCommands.registerCommand("wiggleIntake", AutoCommands.autoIndexerWiggleIntake(intake));
        NamedCommands.registerCommand("index", new SequentialCommandGroup(
            indexer.setTargetVelocityCommand(IndexerValue.INTAKE),
            kicker.setTargetVelocityCommand(KickerValue.INTAKE.getKickerValue())
        ));
        NamedCommands.registerCommand("stopIndexer", new SequentialCommandGroup(
            indexer.setTargetVelocityCommand(IndexerValue.STOP),
            kicker.setTargetVelocityCommand(KickerValue.STOP.getKickerValue())
        ));
        // NamedCommands.registerCommand("resetShooter", new SequentialCommandGroup(
        //     indexer.setTargetVelocityCommand(IndexerValue.STOP.getIndexerValue()),
        //     kicker.setTargetVelocityCommand(KickerValue.STOP.getKickerValue())
        // ));

        
        // addTuningAuto(drive);
        addAutos(drive, intake, indexer, kicker, shooter, vision);
        // addTurretTesting(drive, shooter);

        TelemetryUtils.registerDashboard(this);
    }
    
    public  Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // public static void addTurretTesting(SwerveDrive drive, Shooter shooter) {
    //     autoChooser.addOption("TurretTestStrafeRight", new ParallelCommandGroup(
    //         TuningAutos.strafeRight(drive),
    //         new ShootHub(drive, shooter))
    //     ); // Use this for Turret Testing
    //     autoChooser.addOption("TurretTestStrafeLeft", new ParallelCommandGroup(
    //         TuningAutos.strafeLeft(drive),
    //         new ShootHub(drive, shooter))
    //     ); // Use this for Turret Testing
    // }
    
    public static void addTuningAuto(SwerveDrive drive) {
        autoChooser.addOption("BackTest", TuningAutos.backTest(drive));
        autoChooser.addOption("ForwardTest", TuningAutos.forwardTest(drive));
        autoChooser.addOption("TurnTest", TuningAutos.turnTest(drive));
        autoChooser.addOption("SplineTest", TuningAutos.splineTest(drive));
        autoChooser.addOption("StrafeRight", TuningAutos.strafeRight(drive));
        autoChooser.addOption("StrafeLeft", TuningAutos.strafeLeft(drive));
        autoChooser.addOption("LessForwardTest", TuningAutos.lessForwardTest(drive));
    }

    public static void addAutos(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Vision vision) {
        autoChooser.addOption("rightNeutralOutpost", Autos.rightNeutralOutpost(drive, intake, indexer, kicker, shooter, vision));
        // autoChooser.setDefaultOption("test", Autos.rightNeutralOutpost(drive, intake, indexer, kicker, shooter, vision));
        // autoChooser.addOption("rightNeutralOutpostDouble", Autos.rightNeutralOutpostDouble(drive, intake, indexer, kicker, shooter, vision));
        autoChooser.addOption("leftNeutralDepot", Autos.leftNeutralDepot(drive, intake, indexer, kicker, shooter, vision));
        // autoChooser.addOption("leftNeutralDepotDouble", Autos.leftNeutralDepotDouble(drive, intake, indexer, kicker, shooter, vision));


    }
    
    @Override
    public void elasticInit() {
        SmartDashboard.putData("Autos/AutoChooser", autoChooser);
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    // ShootCommand.java
    public class ShootCommand extends SequentialCommandGroup {
        public ShootCommand(Shooter shooter, Indexer indexer) {
            addCommands(
                
            );
        }
    }
}