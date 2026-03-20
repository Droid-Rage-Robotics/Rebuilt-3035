package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopCommands;
import frc.robot.commands.autos.AutoChooser;
import frc.robot.commands.manual.ManualClimb;
import frc.robot.commands.manual.SwerveDriveTeleop;
import frc.robot.commands.shooter.ShootHub;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.ClimbValue;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Indexer.IndexerValue;
import frc.robot.subsystems.Kicker.KickerValue;
import frc.robot.subsystems.drive.SwerveConfig;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.intake.IntakeWheel;
import frc.robot.subsystems.intake.Pivot;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterWheel;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.shooter.Shooter.ShooterValue;
import frc.utility.DRAreaManager;

public class RobotContainer {
	private final SwerveConfig swerveConfig = new SwerveConfig();
	public final SwerveDrive drive = new SwerveDrive(true, swerveConfig);
	// private final Vision vision = new Vision();
	private final Intake intake = new Intake(
        new Pivot(false),
        new IntakeWheel(false)
    );
    private final Indexer indexer = new Indexer(false);
    private final Kicker kicker = new Kicker(false);
    private final Shooter shooter = new Shooter(
        new Turret(false),
        new Hood(false),
        new ShooterWheel(false)
    );
    private final Climb climb = new Climb(false);

	private final DRAreaManager areaManager = new DRAreaManager(drive);

	private final AutoChooser autoChooser = new AutoChooser(drive, intake, indexer, kicker, shooter, null);

    // private final Light light = new Light(0);
    
    private final CommandXboxController driver =
		new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
	
	private final CommandXboxController operator =		
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);
	
    // private final Telemetry logger = new Telemetry();

	public RobotContainer() {
		// drive.registerTelemetry(logger::telemeterize);
		
		DriverStation.silenceJoystickConnectionWarning(true);
        
		LiveWindow.disableAllTelemetry(); // LiveWindow is not used so disable for performance boost
		
		configureTeleOpBindings();
	}

	public void configureTeleOpBindings() {

		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
		shooter.getShooterWheel().setDefaultCommand(new ShootHub(drive, shooter));
		// light.setDefaultCommand(new LightCommand(light));

		// driver.rightTrigger()
		// 	.onTrue(intake.getPivot().setTargetPositionCommand(Intake.IntakeValue.PivotAngle.DOWN))
		// 	.onTrue(intake.setTargetVelocityWaitCommand(IntakeValue.WheelVelocity.INTAKE))
		// 	.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP));
		// driver.leftTrigger()
		// 	.onTrue(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.OUTTAKE))
		// 	.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP));


		// driver.povUp()
		// 	.onTrue(climb.setTargetPositionCommand(ClimbValue.CLIMB.getHeight()));
		// driver.povDown()
		// 	.onFalse(climb.setTargetPositionCommand(ClimbValue.START.getHeight()));
		
		// shooter.getTurret().setGoalAngle(ControllerUtils.getRightStickRotation2d(operator)); //ToDo: Test


		// climb.setDefaultCommand(new ManualClimb(climb, operator::getLeftY));
		// operator.y()
		// 	.onTrue(shooter.setShooterTargetCommand(ShooterValue.SHORT));
		// operator.b()
		// 	.onTrue(shooter.setShooterTargetCommand(ShooterValue.CORNER_RIGHT));
		// operator.x()
		// 	.onTrue(shooter.setShooterTargetCommand(ShooterValue.CORNER_LEFT));
		// operator.a()
		// 	.onTrue(shooter.setShooterTargetCommand(ShooterValue.HOLD));
		
		// operator.povUp()
		// 	.onTrue(intake.getPivot().setTargetPositionCommand(Intake.IntakeValue.PivotAngle.UP));

		// operator.povRight()
		// 	.onTrue(intake.getPivot().setTargetPositionCommand(Intake.IntakeValue.PivotAngle.HALF));

		// // operator.povDown()
		// // 	.onTrue(shooter.setShooterTargetCommand(ShooterValue.HOARD));

		// operator.rightBumper()
		// 	.onTrue(TeleopCommands.operatorRightBumperOnTrue(shooter, indexer, kicker,intake))
		// 	.whileTrue(TeleopCommands.indexerWiggleIntake(intake))
		// 	.onFalse(TeleopCommands.operatorRightBumperOnFalse(shooter, indexer, kicker,intake));


		// operator.leftBumper()
		// 	.onTrue(indexer.setTargetVelocityCommand(IndexerValue.OUTTAKE.getIndexerValue()))
		// 	.onTrue(kicker.setTargetVelocityCommand(KickerValue.OUTTAKE.getKickerValue()))
		// 	.onFalse(indexer.setTargetVelocityCommand(IndexerValue.STOP.getIndexerValue()))
		// 	.onFalse(kicker.setTargetVelocityCommand(KickerValue.STOP.getKickerValue()));
	}

	public void resetClimb(){
		climb.setResetting(true);
		operator.rightBumper()
			.onTrue(new InstantCommand(()->climb.getMotor().setPower(0.25)))
			.onFalse(new InstantCommand(()->climb.getMotor().setPower(0)));

		operator.leftBumper()
			.onTrue(new InstantCommand(()->climb.getMotor().setPower(-0.25)))
			.onFalse(new InstantCommand(()->climb.getMotor().setPower(0)));
	}

	public void periodic() {
		areaManager.periodic();
	}

	public Command getAutonomousCommand() {
		return autoChooser.getAutonomousCommand();
	}

	public void resetSubsystemsAutoExit() {
		shooter.setShooterTarget(ShooterValue.HOLD);
		indexer.setTargetVelocity(Indexer.IndexerValue.STOP.getIndexerValue());
		kicker.setTargetVelocity(KickerValue.STOP.getKickerValue());
		intake.getIntakeWheel().setTargetVelocity(IntakeValue.WheelVelocity.STOP.getVelocity());
	}
}
