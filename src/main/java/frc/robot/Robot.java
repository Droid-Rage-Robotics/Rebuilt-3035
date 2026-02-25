package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.MatchValue;

public class Robot extends LoggedRobot {
    private final RobotContainer robotContainer = new RobotContainer();

    // private final AutoChooser autoChooser = new AutoChooser(drive, intake, indexer,shooter, light);

    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

    private Command autonomousCommand;

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

        // // Starts recording to data log
        // DataLogManager.start();
        // // Record both DS control and joystick data
        // DriverStation.startDataLog(DataLogManager.getLog());
        
        // vision.setUpVision();
        // SmartDashboard.putData("Robot Misc", DroidRageConstants.robotMisc);

        // DroidRageConstants.alliance = DriverStation.getAlliance().get();
    }
    
    @Override
    public void robotPeriodic() {
        commandScheduler.run();
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
        commandScheduler.cancelAll();

        // SignalLogger.start(); // CTRE Signal Logger

        // autonomousCommand = autoChooser.getAutonomousCommand();
        // autonomousCommand = new InstantCommand();

        if (autonomousCommand != null) {
            commandScheduler.schedule(autonomousCommand);
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
        commandScheduler.cancelAll();
        // SignalLogger.start(); // CTRE Signal Logger
        
        // if (autonomousCommand != null) {
        //     autonomousCommand.cancel();
        // }
		DriverStation.silenceJoystickConnectionWarning(true);
        // robotContainer.configureTeleOpBindings();
        // robotContainer.testDrive();
        robotContainer.testSubsystems();
        robotContainer.testClimb();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit(){
        SignalLogger.stop();
    }
    
    @Override
    public void testInit() {
        commandScheduler.cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}