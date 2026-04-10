package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.MatchValue;

public class Robot extends LoggedRobot {
    private final RobotContainer robotContainer = new RobotContainer();

    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

    private Command autonomousCommand;

    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStarAK());
        commandScheduler.schedule(PathfindingCommand.warmupCommand());;

        if (DriverStation.isFMSAttached()) {
            TelemetryUtils.Config.Match = MatchValue.COMPETITION;
        } else {
            TelemetryUtils.Config.Match = MatchValue.PRACTICE;
        }

        TelemetryUtils.onRobotInit();

        if (DriverStation.getAlliance().isPresent()) {
            DroidRageConstants.alliance = DriverStation.getAlliance().get();
        }

        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
    }
    
    @Override
    public void robotPeriodic() {
        commandScheduler.run();
        TelemetryUtils.onRobotPeriodic();
        robotContainer.periodic();

        // if(DriverStation.isEStopped()){ //Robot Estopped
        //     light.flashingColors(light.red, light.white);
        // }
    }

    @Override
    public void disabledInit() {
        commandScheduler.cancelAll();
    }
    
    @Override
    public void disabledPeriodic() {
        TelemetryUtils.onDisabledPeriodic();
    }

    @Override
    public void autonomousInit() {
        commandScheduler.cancelAll();

        autonomousCommand = robotContainer.getAutonomousCommand();

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

        robotContainer.resetSubsystemsAutoExit();

    }

    @Override
    public void teleopInit() {
        commandScheduler.cancelAll();
        
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        
        DroidRageConstants.alliance = DriverStation.getAlliance().get();


        /* DO NOT INITIALIZE BUTTON BINDINGS HERE */
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