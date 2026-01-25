package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SysID.DriveSysID;
import frc.robot.SysID.SysID;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.Turn180Degrees;
import frc.robot.commands.SysId.SysIdCommand;
import frc.robot.commands.SysId.SysIdRoutineCommand;
import frc.robot.commands.drive.TeleopAlign;
import frc.robot.commands.manual.SwerveDriveTeleop;
import frc.robot.commands.manual.Turning;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
	private final CommandXboxController driver, operator;
	
	public RobotContainer(CommandXboxController driver, CommandXboxController operator){
		DriverStation.silenceJoystickConnectionWarning(true);
		this.driver = driver;
		this.operator = operator;

	}

	public void configureTeleOpBindings(
		SwerveDrive drive,
		Vision vision
		) {
		// // Slow Mode and Gyro Reset in the Default Command
		// drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver, elevator));
		// // drive.setDefaultCommand(new Turning(drive, driver, elevator));
		// // elevator.setDefaultCommand(new ManualElevator(elevator, operator::getRightY));
		// operator.y()
		// 	.onTrue(new TeleopCommands().goL4(elevator, carriage));
		
		


		// rumble.onTrue(
        // new StartEndCommand(
        //         () -> {
        //           driver.setRumble(RumbleType.kLeftRumble, 1.0);
        //           driver.setRumble(RumbleType.kRightRumble, 1.0);
        //         },
        //         () -> {
        //           driver.setRumble(RumbleType.kLeftRumble, 0.0);
        //           driver.setRumble(RumbleType.kRightRumble, 0.0);
        //         })
        //     .withTimeout(0.5));
	}

	public void testDrive(CommandXboxController driver, SwerveDrive drive, Vision vision) {
		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
	}

	public void driveSysID(DriveSysID sysID){
		driver.povUp().whileTrue(sysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		driver.povDown().whileTrue(sysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		driver.povLeft().whileTrue(sysID.sysIdDynamic(SysIdRoutine.Direction.kForward));
		driver.povRight().whileTrue(sysID.sysIdDynamic(SysIdRoutine.Direction.kReverse));
	}

	public void sysID(SysID sysID){
		driver.a().onTrue(new SysIdCommand(sysID));
		driver.b().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
	}
	

}
