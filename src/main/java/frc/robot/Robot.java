package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.SysID.SysID;
// import frc.robot.SysID.SysID.Measurement;
import frc.robot.commands.SysId.ManualSysIdRoutine;
import frc.robot.commands.SysId.SysIdRoutineCommand;
import frc.robot.commands.autos.AutoChooser;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Light;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.Telemetry;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeWheel;
import frc.robot.subsystems.intake.Pivot;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterWheel;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.Vision;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.MatchValue;
import frc.utility.LimelightEx;
import frc.utility.MatchTimerSpeaker;

public class Robot extends LoggedRobot {
    private final Vision vision = new Vision();
    // private final SwerveDrive drive = new SwerveDrive(false, vision);

    // private final Telemetry telemetry = new Telemetry(drive);
    // private final Intake intake = new Intake(
    //     new Pivot(false),
    //     new IntakeWheel(false)
    // );
    // private final Indexer indexer = new Indexer(false);
    // private final Shooter shooter = new Shooter(
    //     new Turret(false),
    //     new Hood(false),
    //     new ShooterWheel(false),
    //     vision
    // );
    // private final Light light = new Light(0);
    
    private final CommandXboxController driver =
		new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
	private final CommandXboxController operator =		
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);

    // private final CycleTracker cycleTracker = new CycleTracker();

    // private final CANdle candle = new CANdle(0);

    // private final DriveSysID driveSysID = new DriveSysID(drive.getSwerveModules(), drive);
    // private final SysID sysID = new SysID(carriage.getIntake().getMotor(), carriage.getIntake());

    private final RobotContainer robotContainer = new RobotContainer(driver, operator);
    // private final AutoChooser autoChooser = new AutoChooser(drive, intake, indexer,shooter, light);

    // public boolean teleopRan;
    private Command autonomousCommand;
    // private MatchTimerSpeaker matchTimeSpeaker = new MatchTimerSpeaker();
    
    // private final CrapTurret crap = new CrapTurret(true);

    // private final LimelightEx crapLimelight = LimelightEx.create("limelight-right");

    @Override
    public void robotInit() {
        // SignalLogger.setPath("/home/lvuser/logs/ctre/");
        if (DriverStation.isFMSAttached()) {
            TelemetryUtils.Config.Match = MatchValue.COMPETITION;
        } else {
            TelemetryUtils.Config.Match = MatchValue.PRACTICE;
        }
        
        TelemetryUtils.onRobotInit();

        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();

        // candle.setControl(new RainbowAnimation(0, 399));

        

        // // Starts recording to data log
        // DataLogManager.start();
        // // Record both DS control and joystick data
        // DriverStation.startDataLog(DataLogManager.getLog());
        
        // vision.setUpVision();
        SmartDashboard.putData("Robot Misc", DroidRageConstants.robotMisc);

        // crap.resetEncoder();

        DroidRageConstants.alliance = DriverStation.getAlliance().get();
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        TelemetryUtils.onRobotPeriodic();

        // if(DriverStation.isEStopped()){ //Robot Estopped
        //     light.flashingColors(light.red, light.white);
        // }
    }

    @Override
    public void disabledInit() {}
    
    @Override
    public void disabledPeriodic() {
        TelemetryUtils.onDisabledPeriodic();
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();

        // SignalLogger.start(); // CTRE Signal Logger

        // autonomousCommand = autoChooser.getAutonomousCommand();
        // autonomousCommand = new InstantCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        // if(DriverStation.isEStopped()){ //Robot Estopped
        //     light.flashingColors(light.red, light.white);
        // }
    }

    @Override
    public void autonomousExit(){
        SignalLogger.stop();
        switch (DriverStation.getGameSpecificMessage()) { // Set didWeWin for Lights
            case "R": // Red won Auto
                DroidRageConstants.didWeWin = DroidRageConstants.alliance == DriverStation.Alliance.Red;
                break;
            case "B": // Blue won Auto
                DroidRageConstants.didWeWin = DroidRageConstants.alliance == DriverStation.Alliance.Blue;
                break;
        }
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        // MatchTimerSpeaker.playSound("songs/plswork.mp3");
        // SignalLogger.start(); // CTRE Signal Logger
        
        // if (autonomousCommand != null) {
        //     autonomousCommand.cancel();
        // }
		DriverStation.silenceJoystickConnectionWarning(true);
        // robotContainer.configureTeleOpBindings(drive, intake, indexer, shooter, vision, light);
        // robotContainer.testDrive(driver, drive, vision);

        
        // robotContainer.resetClimb(climb);
        // vision.setUpVision(); //Has to be here to set up Limelight Pipelines

        // robotContainer.sysID(driveSysID);
        // robotContainer.sysID(sysID);

        // crap.resetEncoder();
    }

    @Override
    public void teleopPeriodic() {
        // if (LimelightHelpers.getTV("limelight-right")) {
        //     double txDeg = LimelightHelpers.getTX("limelight-right");
        //     Rotation2d currentAngle = crap.getCurrentAngle();

        //     // Shift the goal by the Limelight error
        //     Rotation2d newGoal = currentAngle.plus(Rotation2d.fromDegrees(txDeg));
        //     crap.setGoalAngle(newGoal);
        // } else {
        //     crap.setGoalAngle(crap.getCurrentAngle());
        // }
    }

    @Override
    public void teleopExit(){
        SignalLogger.stop();
        // cycleTracker.printAllData();
    }
    
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}