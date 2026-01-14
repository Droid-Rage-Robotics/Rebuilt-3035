package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveDriveConstants;
import frc.utility.DashboardUtils;
import frc.utility.DashboardUtils.Dashboard;

public class AutoChooser implements Dashboard {
    public static final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public AutoChooser(SwerveDrive drive){
        createAutoBuilder(drive);
        
        DashboardUtils.registerDashboard(this);
    }
    
    public  Command getAutonomousCommand() {
        return autoChooser.getSelected();
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

    public static void addAutos() {}

    public static void createAutoBuilder(SwerveDrive drive){
        try {
            // RobotConfig config = new RobotConfig(Units.lbsToKilograms(120), 1, 
            //     null, 
            //     Units.inchesToMeters(29));
            RobotConfig config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder
            AutoBuilder.configure(
                drive::getPose,
                drive::resetOdometry,
                drive::getSpeeds,
                drive::setFeedforwardModuleStates,
                new PPHolonomicDriveController(
                        new PIDConstants(SwerveDriveConstants.SwerveDriveConfig.TRANSLATIONAL_KP.getValue(), 
                    SwerveDriveConstants.SwerveDriveConfig.TRANSLATIONAL_KI.getValue(), 
                    SwerveDriveConstants.SwerveDriveConfig.TRANSLATIONAL_KD.getValue()),  // Translation PID constants
                new PIDConstants(SwerveDriveConstants.SwerveDriveConfig.THETA_KP.getValue(), 
                    SwerveDriveConstants.SwerveDriveConfig.THETA_KI.getValue(), 
                    SwerveDriveConstants.SwerveDriveConfig.THETA_KD.getValue())),  // Rotation PID constants
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drive // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData("Autos/AutoChooser", autoChooser);
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}
}
