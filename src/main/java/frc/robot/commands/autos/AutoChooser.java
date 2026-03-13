package frc.robot.commands.autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TeleopCommands;
import frc.robot.commands.shooter.ShootHub;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.ClimbValue;
import frc.robot.subsystems.Indexer.IndexerValue;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Light;
import frc.robot.subsystems.drive.SwerveDrive;
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

    public AutoChooser(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Climb climb, Vision vision){
        NamedCommands.registerCommand("startPivot", AutoCommands.startPivotCommand(intake));
        NamedCommands.registerCommand("intakeDown",  intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN.getAngle()));
        NamedCommands.registerCommand("intakeUp", intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.UP.getAngle()));

        NamedCommands.registerCommand("climbDown", climb.setTargetPositionCommand(ClimbValue.START.getHeight()));
        NamedCommands.registerCommand("climbUp", climb.setTargetPositionCommand(ClimbValue.CLIMB.getHeight()));

        NamedCommands.registerCommand("intake",
            intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE)
        );

        NamedCommands.registerCommand("outtake", intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.OUTTAKE));
        NamedCommands.registerCommand("shootTrenchR", shooter.setShooterTargetCommand(ShooterValue.SHOOT_TRENCH_RIGHT));
        NamedCommands.registerCommand("intakeWait", intake.setTargetVelocityWaitCommand(WheelVelocity.INTAKE));
        NamedCommands.registerCommand("shootTrenchL", shooter.setShooterTargetCommand(ShooterValue.SHOOT_TRENCH_LEFT));
        NamedCommands.registerCommand("shootOutpost", AutoCommands.shootOutpost(shooter, indexer, kicker));
        NamedCommands.registerCommand("wiggleIntake", getAutonomousCommand());
        NamedCommands.registerCommand("index", indexer.setTargetVelocityCommand(IndexerValue.INTAKE.getIndexerValue()));
        NamedCommands.registerCommand("shoot", new SequentialCommandGroup(
            new ParallelCommandGroup(
            indexer.setTargetVelocityCommand(Indexer.IndexerValue.INTAKE.getIndexerValue()),
            kicker.setTargetVelocityCommand(Kicker.KickerValue.INTAKE.getKickerValue())
            ),
            new WaitCommand(2),
            new ParallelCommandGroup(
                indexer.setTargetVelocityCommand(Indexer.IndexerValue.STOP.getIndexerValue()),
                kicker.setTargetVelocityCommand(Kicker.KickerValue.STOP.getKickerValue())
            )
        ));


        // addTuningAuto(drive);
        addAutos(drive, intake, indexer, kicker, shooter, climb, vision);
        addTurretTesting(drive, shooter);

        TelemetryUtils.registerDashboard(this);
    }
    
    public  Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private static void addTurretTesting(SwerveDrive drive, Shooter shooter) {
        autoChooser.addOption("TurretTestStrafeRight", new ParallelCommandGroup(
            TuningAutos.strafeRight(drive),
            new ShootHub(drive, shooter))
        ); // Use this for Turret Testing
        autoChooser.addOption("TurretTestStrafeLeft", new ParallelCommandGroup(
            TuningAutos.strafeLeft(drive),
            new ShootHub(drive, shooter))
        ); // Use this for Turret Testing
    }
    
    public static void addTuningAuto(SwerveDrive drive) {
        autoChooser.addOption("BackTest", TuningAutos.backTest(drive));
        autoChooser.addOption("ForwardTest", TuningAutos.forwardTest(drive));
        autoChooser.addOption("TurnTest", TuningAutos.turnTest(drive));
        autoChooser.addOption("SplineTest", TuningAutos.splineTest(drive));
        autoChooser.addOption("StrafeRight", TuningAutos.strafeRight(drive));
        autoChooser.addOption("StrafeLeft", TuningAutos.strafeLeft(drive));
        autoChooser.addOption("LessForwardTest", TuningAutos.lessForwardTest(drive));
        // autoChooser.addOption("ForwardAndBack", TuningAutos.forwardAndBackTest(drive));
    }

    public static void addAutos(SwerveDrive drive, Intake intake, Indexer indexer, Kicker kicker, Shooter shooter, Climb climb, Vision vision) {
        autoChooser.addOption("rightNeutralOutpost", Autos.rightNeutralOutpost(drive, intake, indexer, kicker, shooter, vision));
        autoChooser.addOption("rightCrossNeutral", Autos.rightCrossNeutral(drive, intake, indexer, kicker, shooter, vision));
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

