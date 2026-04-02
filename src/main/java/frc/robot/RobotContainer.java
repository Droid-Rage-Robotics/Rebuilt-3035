package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.drive.SwerveDrive;
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
import frc.robot.subsystems.shooter.ShooterWheel;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.shooter.Shooter.ShooterValue;
import frc.utility.DRAreaManager;

public class RobotContainer {
	private final SwerveConfig swerveConfig = new SwerveConfig();
	public final SwerveDrive drive = new SwerveDrive(true, swerveConfig);
	private final Vision vision = new Vision();
	private final Intake intake = new Intake(
        new Pivot(true),
        new IntakeWheel(true)
    );
    // private final Indexer indexer = new Indexer(true);
	private final Indexer indexer = new Indexer(
		new BottomRollers(true), 
		new TopRoller(true)
	);

    private final Kicker kicker = new Kicker(true);
    private final Shooter shooter = new Shooter(
        new Turret(true),
        new Hood(true),
        new ShooterWheel(true)
    );

	private final DRAreaManager areaManager = new DRAreaManager(drive);

	private final AutoChooser autoChooser = new AutoChooser(drive, intake, indexer, kicker, shooter, vision);

    // private final Light light = new Light(0);
    
    private final CommandXboxController driver =
		new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
	
	private final CommandXboxController operator =		
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);
	

	public RobotContainer() {
		// drive.registerTelemetry(logger::telemeterize);
		DriverStation.silenceJoystickConnectionWarning(true);
        
		LiveWindow.disableAllTelemetry(); // LiveWindow is not used so disable for performance boost
		
		configureTeleOpBindings();
		// operator.a().onTrue(indexer.getBottomRollers().getSysIdCommand());
		// operator.a().onTrue(indexer.getTopRoller().getSysIdCommand());

		// testShootingMove();
	}

	public void configureTeleOpBindings() {
		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
		shooter.getShooterWheel().setDefaultCommand(new DRShooter(drive, shooter));
		// light.setDefaultCommand(new LightCommand(light));
		
		driver.a()
			.onTrue(
				new InstantCommand(()-> drive.resetPose(
					new Pose2d(3.5,0.5,new Rotation2d(
						0
					))
				))
			);
		

		driver.rightTrigger()
			.onTrue(intake.getPivot().setTargetPositionCommand(Intake.IntakeValue.PivotAngle.DOWN))
			.whileTrue(intake.setTargetVelocityWaitCommand(IntakeValue.WheelVelocity.INTAKE))
			.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP));
		driver.leftTrigger()
			.onTrue(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.OUTTAKE))
			.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP));

		operator.y()
			.onTrue(shooter.setShooterTargetCommand(ShooterValue.SHORT));
		operator.b()
			.onTrue(shooter.setShooterTargetCommand(ShooterValue.SHOOT_TRENCH_RIGHT)); //CORNER_RIGHT
		operator.x()
			.onTrue(shooter.setShooterTargetCommand(ShooterValue.SHOOT_TRENCH_LEFT)); //ORNER_LEFT
		operator.a()
			.onTrue(shooter.setShooterTargetCommand(ShooterValue.HOLD));
		
		operator.povUp()
			.onTrue(intake.getPivot().setTargetPositionCommand(Intake.IntakeValue.PivotAngle.UP));

		operator.povRight()
			.onTrue(intake.getPivot().setTargetPositionCommand(Intake.IntakeValue.PivotAngle.HALF));

		operator.povDown()
			.onTrue(new InstantCommand(()->DroidRageConstants.isShooterManual = false));

		operator.rightBumper()
			.whileTrue(TeleopCommands.operatorRightBumperOnTrue(indexer, kicker,intake))
			.whileTrue(TeleopCommands.indexerWiggleIntake(intake))
			.onFalse(TeleopCommands.operatorRightBumperOnFalse(indexer, kicker,intake));


		operator.leftBumper()
			.onTrue(indexer.setTargetVelocityCommand(IndexerValue.OUTTAKE))
			.onTrue(kicker.setTargetVelocityCommand(KickerValue.OUTTAKE.getKickerValue()))
			.onFalse(indexer.setTargetVelocityCommand(IndexerValue.STOP))
			.onFalse(kicker.setTargetVelocityCommand(KickerValue.STOP.getKickerValue()));
	}

	public void testShootingMove(){
		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
		shooter.getShooterWheel().setDefaultCommand(new DRShooter(drive, shooter));
		
		// shooter.getHood().setDefaultCommand(new ManualHood(shooter, operator));
		// shooter.getShooterWheel().setDefaultCommand(new ManualShooterWheel(shooter, operator));
		operator.rightBumper()
			.onTrue(TeleopCommands.operatorRightBumperOnTrue(indexer, kicker,intake))
			// .whileTrue(TeleopCommands.indexerWiggleIntake(intake))
			.onFalse(TeleopCommands.operatorRightBumperOnFalse(indexer, kicker,intake));
		operator.leftBumper()
			.onTrue(indexer.setTargetVelocityCommand(IndexerValue.OUTTAKE))
			.onTrue(kicker.setTargetVelocityCommand(KickerValue.OUTTAKE.getKickerValue()))
			.onFalse(indexer.setTargetVelocityCommand(IndexerValue.STOP))
			.onFalse(kicker.setTargetVelocityCommand(KickerValue.STOP.getKickerValue()));

		driver.a()
			.onTrue(
				new InstantCommand(()-> drive.resetPose(
					new Pose2d(3.5,0.5,new Rotation2d(
						0
					))
				))
			);
		
		// operator.povUp()
		// 	.whileTrue(
		// 		shooter.getHood().setTargetPositionCommand(Rotation2d.fromDegrees(shooter.getHood().getGoalAngle().getDegrees()+2))
		// 	);
		// operator.povDown()
		// 	.whileTrue(
		// 		shooter.getHood().setTargetPositionCommand(Rotation2d.fromDegrees(shooter.getHood().getGoalAngle().getDegrees()-2))
		// 	);

		// operator.povRight()
		// 	.whileTrue(
		// 		shooter.getShooterWheel().setTargetVelocityCommand(
		// 			RotationsPerSecond.of(shooter.getShooterWheel().getTargetVelocity().magnitude()+5))
		// 	);
		// operator.povLeft()
		// 	.whileTrue(
		// 		shooter.getShooterWheel().setTargetVelocityCommand(
		// 			RotationsPerSecond.of(shooter.getShooterWheel().getTargetVelocity().magnitude()-5))
		// 	);
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
