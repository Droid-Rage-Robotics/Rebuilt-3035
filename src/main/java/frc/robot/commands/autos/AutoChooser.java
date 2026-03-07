package frc.robot.commands.autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShooterScore;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Light;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.Dashboard;

public class AutoChooser implements Dashboard {
    public static final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public AutoChooser(SwerveDrive drive, Intake intake, Indexer indexer, Shooter shooter, Light light, Climb climb){
        NamedCommands.registerCommand("intakeDown", AutoCommands.intakeDown(intake));
        NamedCommands.registerCommand("intakeUp", AutoCommands.intakeUp(intake));
        NamedCommands.registerCommand("climbDown", AutoCommands.climbDown(climb));
        NamedCommands.registerCommand("climbUp", AutoCommands.climbUp(climb));
        NamedCommands.registerCommand("intake", AutoCommands.intake(intake));
        NamedCommands.registerCommand("outtake", AutoCommands.outtake(intake));
        NamedCommands.registerCommand("shootFromBlue", AutoCommands.shootFromBlue(shooter));
        NamedCommands.registerCommand("shootFromRed", AutoCommands.shootFromRed(shooter));

        // addTuningAuto(drive);
        addTurretTesting(drive, shooter);

        TelemetryUtils.registerDashboard(this);
    }
    
    public  Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private static void addTurretTesting(SwerveDrive drive, Shooter shooter) {
        autoChooser.addOption("TurretTestStrafeRight", new ParallelCommandGroup(
            TuningAutos.strafeRight(drive),
            new ShooterScore(drive, shooter))
        ); // Use this for Turret Testing
        autoChooser.addOption("TurretTestStrafeLeft", new ParallelCommandGroup(
            TuningAutos.strafeLeft(drive),
            new ShooterScore(drive, shooter))
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

    public static void addMiddleAuto(SwerveDrive drive, Intake intake, Indexer indexer, Shooter shooter, Light light){

    }

    public static void addAutos() {}
    
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

