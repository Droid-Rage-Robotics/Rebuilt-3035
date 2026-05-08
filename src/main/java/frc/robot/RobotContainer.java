package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopCommands;
import frc.robot.commands.autos.AutoChooser;
import frc.robot.commands.manual.SwerveDriveTeleop;
import frc.robot.commands.shooter.DRShooter;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Kicker.KickerValue;
import frc.robot.subsystems.drive.SwerveConfig;
import frc.robot.subsystems.drive.maple.Drive;
import frc.robot.subsystems.indexer.BottomRollers;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerValue;
import frc.robot.subsystems.indexer.TopRoller;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.intake.IntakeWheel;
import frc.robot.subsystems.intake.Pivot;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterValue;
import frc.robot.subsystems.shooter.ShooterWheel;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.vision.maple.Vision;
// import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.maple.VisionConstants;
import frc.utility.DRAreaManager;
import frc.utility.io.devices.GyroIOPigeon2;
import frc.utility.io.devices.VisionIOLimelight;
import frc.utility.io.swerve.ModuleIOTalonFXReal;
import lombok.Getter;

public class RobotContainer {
	private final CommandXboxController driver =
		new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
	
	private final CommandXboxController operator =		
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);
	
	@Getter private static final SwerveConfig swerveConfig = new SwerveConfig();
	// public final SwerveDrive drive = new SwerveDrive(true, swerveConfig);
	private final Drive drive;
	// private final Vision vision = new Vision();
	private final Vision vision;
	private final Intake intake = new Intake(
        new Pivot(false, driver),
        new IntakeWheel(false)
    );
	private final Indexer indexer = new Indexer(
		new BottomRollers(false), 
		new TopRoller(false)
	);

    private final Kicker kicker = new Kicker(false);
    private final Shooter shooter = new Shooter( 
        new Turret(false),
        new Hood(false),
        new ShooterWheel(false)
    );

	private final DRAreaManager areaManager;

	private final AutoChooser autoChooser;

	public RobotContainer() {
		// Real robot, instantiate hardware IO implementations
                drive = new Drive(false,
					new GyroIOPigeon2(),
					new ModuleIOTalonFXReal(swerveConfig.getFrontLeft()),
					new ModuleIOTalonFXReal(swerveConfig.getFrontRight()),
					new ModuleIOTalonFXReal(swerveConfig.getBackLeft()),
					new ModuleIOTalonFXReal(swerveConfig.getBackRight()),
                        (pose) -> {});
                this.vision = new Vision(
					drive,
					new VisionIOLimelight(VisionConstants.leftLL, drive::getRotation),
					new VisionIOLimelight(VisionConstants.rightLL, drive::getRotation),
					new VisionIOLimelight(VisionConstants.middleLL, drive::getRotation));

				this.areaManager = new DRAreaManager(drive);
				autoChooser = new AutoChooser(drive, intake, indexer, kicker, shooter, vision);
		DriverStation.silenceJoystickConnectionWarning(true);
        
		LiveWindow.disableAllTelemetry(); // LiveWindow is not used so disable for performance boost
		
		configureTeleOpBindings();
		// driver.a().onTrue(Drive.feedforwardCharacterization(drive));
		// testShootingMove();
		// driver.a().onTrue(shooter.getTurret().getSysIdCommand());
		// driver.a().onTrue(shooter.getShooterWheel().getSysIdCommand());
	}

	public void configureTeleOpBindings() {
		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
		shooter.getShooterWheel().setDefaultCommand(new DRShooter(drive, shooter));
		
		// driver.povRight()
		// 	.onTrue(new InstantCommand(() -> {
		// 		shooter.getHood().getMotor().setPower(1);
		// 		shooter.getHood().disableControlLoop();
		// 		})
		// 	)
		// 	.onFalse(shooter.getHood().enableControlLoop());

		// driver.povLeft()
		// 	.onTrue(new InstantCommand(() -> {
		// 		shooter.getHood().getMotor().setPower(1);
		// 		shooter.getHood().disableControlLoop();
		// 		})
		// 	)
		// 	.onFalse(shooter.getHood().enableControlLoop());

		driver.rightTrigger()
			.onTrue(intake.getPivot().setTargetPositionCommand(Intake.IntakeValue.PivotAngle.DOWN))
			.whileTrue(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE))
			.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP));
		
		driver.leftTrigger()
			.whileTrue(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.OUTTAKE))
			.whileTrue(indexer.setTargetVelocityCommand(IndexerValue.OUTTAKE))
			.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP))
			.onFalse(indexer.setTargetVelocityCommand(IndexerValue.STOP));

		operator.y()
			.onTrue(shooter.setShooterTargetCommand(ShooterValue.SHORT));
		operator.b()
			.onTrue(shooter.setShooterTargetCommand(ShooterValue.SHOOT_TRENCH_RIGHT)); 
			// .onTrue(shooter.setShooterTargetCommand(ShooterValue.CORNER_RIGHT));

		operator.x()
			.onTrue(shooter.setShooterTargetCommand(ShooterValue.SHOOT_TRENCH_LEFT));
			// .onTrue(shooter.setShooterTargetCommand(ShooterValue.CORNER_LEFT));

		operator.a()
			.onTrue(shooter.setShooterTargetCommand(ShooterValue.HOLD));
		
		driver.povUp()
			.onTrue(intake.getPivot().setTargetPositionCommand(Intake.IntakeValue.PivotAngle.UP));

		operator.povDown()
			.onTrue(new InstantCommand(()->DroidRageConstants.isShooterManual = false));

		operator.povLeft().and(()->shooter.isShooterReady())
			.whileTrue(TeleopCommands.operatorPovLeftWhileTrue(indexer, kicker,intake,shooter))
			// .onTrue(drive.scaleStator(0.3))
			// .onFalse(drive.disableTurboTorque())
			// .onTrue(drive.setSpeed(Speed.SLOW))
			// .onFalse(drive.setSpeed(Speed.NORMAL))
			.onFalse(TeleopCommands.operatorRightBumperOnFalse(indexer, kicker,intake));

		operator.rightBumper()
			.whileTrue(TeleopCommands.operatorRightBumperWhileTrue(indexer,kicker,intake))
			.whileTrue(TeleopCommands.indexerWiggleIntake(intake))
			.onFalse(TeleopCommands.operatorRightBumperOnFalse(indexer, kicker,intake));


		operator.leftBumper()
			.onTrue(indexer.setTargetVelocityCommand(IndexerValue.OUTTAKE))
			.onTrue(kicker.setTargetVelocityCommand(KickerValue.OUTTAKE.getKickerValue()))
			.onFalse(indexer.setTargetVelocityCommand(IndexerValue.STOP))
			.onFalse(kicker.setTargetVelocityCommand(KickerValue.STOP.getKickerValue()));
	}

	public void periodic() {
		areaManager.periodic();
	}

	public Command getAutonomousCommand() {
		return autoChooser.getAutonomousCommand();
	}

	public void resetSubsystemsAutoExit() {
		shooter.setShooterTarget(ShooterValue.HOLD);
		indexer.setTargetVelocity(Indexer.IndexerValue.STOP);
		kicker.setTargetVelocity(KickerValue.STOP.getKickerValue());
		intake.getIntakeWheel().setTargetVelocity(IntakeValue.WheelVelocity.STOP.getVelocity());
	}
}
