package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.hardware.CANdle;

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
import frc.robot.SysID.SysID;
import frc.robot.SysID.SysID.Measurement;
import frc.robot.commands.SysId.ManualSysIdRoutine;
import frc.robot.commands.SysId.SysIdRoutineCommand;
import frc.robot.commands.autos.AutoChooser;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.utility.DashboardUtils;
import frc.utility.DashboardUtils.MatchValue;

public class Robot extends TimedRobot {
    private final Vision vision = new Vision();
    private final SwerveDrive drive = new SwerveDrive(true, vision);//-10 Works
    private final CANdle candle = new CANdle(0);
    // private final Elevator elevator = new Elevator(true);
    // private final Carriage carriage = new Carriage(
    //     new Arm(true),
    //     new Pivot(true),
    //     new Intake(true)
    // );
    
    // private Climb climb = new Climb(false);

    private final CommandXboxController driver =
		new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
	
	private final CommandXboxController operator =		
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);

    // private final CycleTracker cycleTracker = new CycleTracker();
    // private final Light light = new Light();*

    // private final DriveSysID driveSysID = new DriveSysID(drive.getSwerveModules(), drive);
    // private final SysID sysID = new SysID(carriage.getIntake().getMotor(), carriage.getIntake());

    private final RobotContainer robotContainer = new RobotContainer(driver, operator);
    // private final AutoChooser autoChooser = new AutoChooser(drive, elevator, carriage, vision);

    // public boolean teleopRan;
    private Command autonomousCommand;
  
    @Override
    public void robotInit() {
        candle.setControl(new RainbowAnimation(8, 67));
        SignalLogger.setPath("/home/lvuser/logs/ctre/");
        DashboardUtils.Config.Match = MatchValue.PRACTICE;
        DashboardUtils.onRobotInit();

        // // Starts recording to data log
        // DataLogManager.start();
        // // Record both DS control and joystick data
        // DriverStation.startDataLog(DataLogManager.getLog());
        
        // vision.setUpVision();
        SmartDashboard.putData("Robot Misc", DroidRageConstants.robotMisc);
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        DashboardUtils.onRobotPeriodic();
        // if(DriverStation.isEStopped()){ //Robot Estopped
        //     light.flashingColors(light.red, light.white);
        // }
    }

    @Override
    public void disabledInit() {}
    
    @Override
    public void disabledPeriodic() {
        DashboardUtils.onDisabledPeriodic();
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();

        // SignalLogger.start(); // CTRE Signal Logger

        // autonomousCommand = autoChooser.getAutonomousCommand();
        // autonomousCommand = new InstantCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
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
        if (autonomousCommand != null) {
        autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        // elevator.resetEncoder();

        SignalLogger.start(); // CTRE Signal Logger
        
        // if (autonomousCommand != null) {
        //     autonomousCommand.cancel();
        // }
		DriverStation.silenceJoystickConnectionWarning(true);

        // drive.changeAllianceRotation(); // NO USE
        
        // robotContainer.configureTeleOpBindings(drive, elevator, carriage, climb, vision);
        // robotContainer.configureTeleOpBindings(drive, elevator, carriage, vision);

        robotContainer.testDrive(driver, drive, vision);

        
        // robotContainer.resetClimb(climb);
        vision.setUpVision(); //Has to be here to set up Limelight Pipelines

        // robotContainer.sysID(driveSysID);
        // robotContainer.sysID(sysID);
        // robotContainer.sysID(driver, elevator.getSysIdRoutine(),carriage);
    }

    @Override
    public void teleopPeriodic() {
        // matchTime.set(DriverStation.getMatchTime());

        // while(true){
		// 	// new OperatorXboxControllerRumble(driver, RumbleType.kBothRumble, 2, 1);
   		// 	driver.getHID().setRumble(RumbleType.kBothRumble, 0);
        
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