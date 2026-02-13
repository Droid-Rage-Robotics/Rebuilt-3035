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
// import frc.robot.SysID.DriveSysID;
// import frc.robot.SysID.SysID;
import frc.robot.commands.LightCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.Turn180Degrees;
// import frc.robot.commands.SysId.SysIdCommand;
// import frc.robot.commands.SysId.SysIdRoutineCommand;
// import frc.robot.commands.drive.TeleopAlign;
import frc.robot.commands.manual.SwerveDriveTeleop;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerValue;
import frc.robot.subsystems.Light;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.shooter.Kicker;
import frc.robot.subsystems.shooter.Kicker.KickerValue;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
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
		Intake intake,
		Climb climb,
		Indexer indexer,
		Kicker kicker,
		Shooter shooter,
		Vision vision,
		Light light
		) {
			// Slow Mode and Gyro Reset in the Default Command
			drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
			// drive.setDefaultCommand(new Turning(drive, driver));
			// climb.setDefaultCommand(new ClimbTeleop(climb, operator::getRightY));
			// climb.setDefaultCommand(new ManualClimb(climb, operator::getLeftY));

			// light.setDefaultCommand(new LightCommand(light));
			driver.rightTrigger()
				.onTrue(intake.setPositionCommand(IntakeValue.INTAKE))
				.onFalse(intake.setPositionCommand(IntakeValue.STOP));
			driver.leftTrigger()
				.onTrue(intake.setPositionCommand(IntakeValue.OUTTAKE))
				.onFalse(intake.setPositionCommand(IntakeValue.STOP));

			// operator.rightTrigger()
    		// 	.onTrue(climb.setClimbPositionCommand(ClimbValue.CLIMB))
    		// 	.onFalse(climb.setClimbPositionCommand(ClimbValue.START));
    			
			// operator.rightBumper()
    		// 	.onTrue(indexer.setIndexerCommand(IndexerValue.INTAKE))
   			// 	.onFalse(indexer.setIndexerCommand(IndexerValue.STOP));

			// operator.leftBumper()
			// 	.onTrue(kicker.setKickerCommand(KickerValue.INTAKE))
			// 	.onFalse(indexer.setKickerCommand(IndexerValue.STOP));

			operator.a()
    			.onTrue(shooter.setShooterModeCommand(ShooterMode.HOLD)); //LED strip: indicate the mode, one automatic (automates itself), three positions
			operator.b()
    			.onTrue(shooter.setShooterModeCommand(ShooterMode.OPPOSITE));
			operator.x()
				.onTrue(shooter.setShooterModeCommand(ShooterMode.SCORE));
			operator.y()
				.onTrue(shooter.setShooterModeCommand(ShooterMode.HOARD));
				
	}

	public void testDrive(CommandXboxController driver, SwerveDrive drive, Vision vision) {
		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
	}

	// public void driveSysID(DriveSysID sysID){
	// 	driver.povUp().whileTrue(sysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
	// 	driver.povDown().whileTrue(sysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
	// 	driver.povLeft().whileTrue(sysID.sysIdDynamic(SysIdRoutine.Direction.kForward));
	// 	driver.povRight().whileTrue(sysID.sysIdDynamic(SysIdRoutine.Direction.kReverse));
	// }

	// public void sysID(SysID sysID){
	// 	driver.a().onTrue(new SysIdCommand(sysID));
	// 	driver.b().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
	// }
	

}
