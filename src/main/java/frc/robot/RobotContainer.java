package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants.FieldConstants;
import frc.robot.commands.TeleopCommands;
import frc.robot.commands.autos.AutoChooser;
import frc.robot.commands.manual.ManualHood;
import frc.robot.commands.manual.ManualShooterWheel;
import frc.robot.commands.manual.SwerveDriveTeleop;
import frc.robot.commands.shooter.DRShooter;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Kicker.KickerValue;
import frc.robot.subsystems.drive.SwerveConfig;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveConfig.Speed;
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
import frc.robot.subsystems.vision.Vision;
import frc.utility.DRAreaManager;

public class RobotContainer {
	private final SwerveConfig swerveConfig = new SwerveConfig();
	public final SwerveDrive drive = new SwerveDrive(true, swerveConfig);
	private final Vision vision = new Vision();
	private final Intake intake = new Intake(
        new Pivot(false),
        new IntakeWheel(false)
    );
    // private final Indexer indexer = new Indexer(true);
	private final Indexer indexer = new Indexer(
		new BottomRollers(false), 
		new TopRoller(false)
	);

    private final Kicker kicker = new Kicker(false);
    private final Shooter shooter = new Shooter(
        new Turret(true),
        new Hood(false),
        new ShooterWheel(false)
    );

	private final DRAreaManager areaManager = new DRAreaManager(drive);

	private final AutoChooser autoChooser = new AutoChooser(drive, intake, indexer, kicker, shooter, vision);

    // private final Light light = new Light(0);
    
    private final CommandXboxController driver =
		new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
	
	private final CommandXboxController operator =		
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);
	

	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);
        
		LiveWindow.disableAllTelemetry(); // LiveWindow is not used so disable for performance boost
		
		configureTeleOpBindings();
		// testShootingMove();
		// driver.a().onTrue(shooter.getTurret().getSysIdCommand());
		// testCurrentChangingLimits();
	}

	public void configureTeleOpBindings() {
		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
		shooter.getShooterWheel().setDefaultCommand(new DRShooter(drive, shooter));
		// light.setDefaultCommand(new LightCommand(light));
		// if(DroidRageConstants.alliance==Alliance.Blue){
			// driver.a()
			// .onTrue(
			// 	new InstantCommand(()-> drive.resetPose(
			// 		new Pose2d(3.534, 3.977, new Rotation2d(
			// 			0
			// 		))
			// 	))
			// );
		// } 
		// else {
		// 	driver.a()
		// 	.onTrue(
		// 		new InstantCommand(()-> drive.resetPose(
		// 			new Pose2d(13, 3.977, new Rotation2d(
		// 				0
		// 			))
		// 		))
		// 	);
		// }
		// driver.a()
		// 	.onTrue(
		// 		new InstantCommand(()-> drive.resetPose(
		// 			new Pose2d(3.534, 3.977, new Rotation2d(
		// 				0
		// 			))
		// 		))
		// 	);
		

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
			// .onTrue(shooter.setShooterTargetCommand(ShooterValue.CORNER_RIGHT)); //CORNER_RIGHT

		operator.x()
			.onTrue(shooter.setShooterTargetCommand(ShooterValue.SHOOT_TRENCH_LEFT)); //ORNER_LEFT
			// .onTrue(shooter.setShooterTargetCommand(ShooterValue.CORNER_LEFT)); //ORNER_LEFT

		operator.a()
			.onTrue(shooter.setShooterTargetCommand(ShooterValue.HOLD));
		
		operator.povUp()
			.onTrue(intake.getPivot().setTargetPositionCommand(Intake.IntakeValue.PivotAngle.UP));

		operator.povRight()
			.onTrue(intake.getPivot().setTargetPositionCommand(Intake.IntakeValue.PivotAngle.HALF));

		operator.povDown()
			.onTrue(new InstantCommand(()->DroidRageConstants.isShooterManual = false));

		
		operator.povLeft()
			.whileTrue(TeleopCommands.operatorRightBumperOnTrue(indexer, kicker,intake))
			.onTrue(drive.setSpeed(Speed.SUPER_SLOW))
			.onFalse(TeleopCommands.operatorRightBumperOnFalse(indexer, kicker,intake));

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

	public void testCurrentChangingLimits(){
		driver.rightTrigger()
			.onTrue(intake.getPivot().setTargetPositionCommand(Intake.IntakeValue.PivotAngle.DOWN))
			.onTrue(new InstantCommand(()->intake.getIntakeWheel().getMotor().turnCurrentLimitOn()))
			.whileTrue(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE))
			.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP));
		driver.leftTrigger()
			.whileTrue(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.OUTTAKE))
			.onTrue(new InstantCommand(()->intake.getIntakeWheel().getMotor().turnCurrentLimitOn()))
			.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP));
		driver.rightBumper()
			.onTrue(intake.getPivot().setTargetPositionCommand(Intake.IntakeValue.PivotAngle.DOWN))
			.onTrue(new InstantCommand(()->intake.getIntakeWheel().getMotor().turnCurrentLimitOff()))
			.whileTrue(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE))
			.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP));
		driver.leftBumper()
			.whileTrue(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.OUTTAKE))
			.onTrue(new InstantCommand(()->intake.getIntakeWheel().getMotor().turnCurrentLimitOff()))
			.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP));
	}

	public void periodic() {
		areaManager.periodic();
		// double distanceRobotToGoal = DRShooter.getDistanceToHub(drive.getState().Pose, FieldConstants.HUB_RED);//TODO: Output Distance
        // System.out.println(distanceRobotToGoal);
		// System.out.println(Shooter.ShooterValue.SHOOT_TRENCH_RIGHT.getTurretAngle().in(Degrees));
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
