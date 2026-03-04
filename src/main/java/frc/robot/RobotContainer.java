package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.manual.ManualClimb;
import frc.robot.commands.manual.SwerveDriveTeleop;
import frc.robot.commands.shooter.ShooterHold;
import frc.robot.commands.shooter.ShooterScore;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.ClimbValue;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerValue;
import frc.robot.subsystems.drive.SwerveConfig;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.intake.IntakeWheel;
import frc.robot.subsystems.intake.Pivot;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Kicker;
import frc.robot.subsystems.shooter.Kicker.KickerValue;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterWheel;
import frc.robot.subsystems.shooter.Turret;

public class RobotContainer {
	private final SwerveConfig swerveConfig = new SwerveConfig();
	public final SwerveDrive drive = new SwerveDrive(true, swerveConfig);
	// private final Vision vision = new Vision();
	private final Intake intake = new Intake(
        new Pivot(true),
        new IntakeWheel(true)
    );
    private final Indexer indexer = new Indexer(false);
    private final Kicker kicker = new Kicker(false);
    private final Shooter shooter = new Shooter(
        new Turret(false),
        new Hood(false),
        new ShooterWheel(false)
    );
    private final Climb climb = new Climb(false);
    // private final Light light = new Light(0);
    
    private final CommandXboxController driver =
		new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
	
	private final CommandXboxController operator =		
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);
	
    private final Telemetry logger = new Telemetry();
	
	

	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);
	}

	public void configureTeleOpBindings() {
		drive.registerTelemetry(logger::telemeterize);

		// Slow Mode and Gyro Reset in the Default Command
		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
		// drive.setDefaultCommand(new Turning(drive, driver));
		climb.setDefaultCommand(new ManualClimb(climb, operator::getLeftY));
		// light.setDefaultCommand(new LightCommand(light));

		driver.rightTrigger()
			.onTrue(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE))
			.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP));
		driver.leftTrigger()
			.onTrue(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.OUTTAKE))
			.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP));


		operator.rightTrigger()
			.onTrue(climb.setTargetPositionCommand(ClimbValue.CLIMB.getHeight()));
		operator.leftTrigger()
			.onFalse(climb.setTargetPositionCommand(ClimbValue.START.getHeight()));
		
		// shooter.getTurret().setGoalAngle(ControllerUtils.getRightStickRotation2d(operator)); //ToDo: Test

		// operator.a()
		// 	.onTrue(shooter.setShooterModeCommand(ShooterMode.HOLD)); //LED strip: indicate the mode, one automatic (automates itself), three positions
		// operator.b()
		// 	.onTrue(shooter.setShooterModeCommand(ShooterMode.OPPOSITE));
		// operator.x()
		// 	.onTrue(shooter.setShooterModeCommand(ShooterMode.SCORE));
		// operator.y()
		// 	.onTrue(shooter.setShooterModeCommand(ShooterMode.HOARD));

		// operater.x()
		// 	.
				
	}

	public void testAim() {
		driver.a()
			.onTrue(new ShooterScore(drive, shooter))
			.onFalse(new ShooterHold(drive, shooter));
	}

	public void testSubsystems() {
		drive.registerTelemetry(logger::telemeterize);
		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
		
		//USE this for TESTING
		


		driver.a()
			.onTrue(new ShooterScore(drive, shooter))
			.onFalse(new ShooterHold(drive, shooter));

		operator.b()
			.onTrue(climb.setTargetPositionCommand(ClimbValue.CLIMB.getHeight()))
			.onFalse(climb.setTargetPositionCommand(ClimbValue.START.getHeight()));
			
		operator.rightBumper()
			.onTrue(indexer.setTargetVelocityCommand(IndexerValue.INTAKE.getIndexerValue()))
			.onFalse(indexer.setTargetVelocityCommand(IndexerValue.STOP.getIndexerValue()));

		operator.leftBumper()
			.onTrue(indexer.setTargetVelocityCommand(IndexerValue.OUTTAKE.getIndexerValue()))
			.onFalse(indexer.setTargetVelocityCommand(IndexerValue.STOP.getIndexerValue()));

		// operator.rightBumper()
		// 	.onTrue(kicker.setTargetVelocityCommand(KickerValue.OUTTAKE.getKickerValue()))
		// 	.onFalse(kicker.setTargetVelocityCommand(KickerValue.STOP.getKickerValue()));

		driver.rightTrigger()
			.onTrue(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.INTAKE))
			.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP));
		driver.leftTrigger()
			.onTrue(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.OUTTAKE))
			.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.WheelVelocity.STOP));

		operator.x()
			.onTrue(intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.DOWN));

		operator.povDown().onTrue(intake.getPivot().setTargetPositionCommand(IntakeValue.PivotAngle.UP));


		// operator.a()
		// 	.onTrue(shooter.setShooterModeCommand(ShooterMode.HOLD)); //LED strip: indicate the mode, one automatic (automates itself), three positions
		// operator.b()
		// 	.onTrue(shooter.setShooterModeCommand(ShooterMode.OPPOSITE));
		// operator.x()
		// 	.onTrue(shooter.setShooterModeCommand(ShooterMode.SCORE));
		// operator.y()
		// 	.onTrue(shooter.setShooterModeCommand(ShooterMode.HOARD));
				
	}

	public void testDrive() {
		drive.registerTelemetry(logger::telemeterize);
		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));

		// driver.back().and(driver.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drive.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
	}
	
	public void testClimb() {
        operator.rightBumper()
            .onTrue(climb.setTargetPositionCommand(Inches.of(climb.getGoalPosition().magnitude() + 0.5)));
        operator.leftBumper()
            .onTrue(climb.setTargetPositionCommand(Inches.of(climb.getGoalPosition().magnitude() - 0.5)));
	}
	public void testShooter() {
		// drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
		// operator.a().onTrue(new ShooterScore(drive, shooter));
		
		// operator.rightBumper()
       	//  	.onTrue(shooter.getShooterWheel().setTargetVelocityCommand(RotationsPerSecond.of(5)))
       	//  	.onFalse(shooter.getShooterWheel().setTargetVelocityCommand(RotationsPerSecond.zero()));

   	 	operator.rightBumper()
			.onTrue(indexer.setTargetVelocityCommand(IndexerValue.INTAKE.getIndexerValue()))
			.onFalse(indexer.setTargetVelocityCommand(IndexerValue.STOP.getIndexerValue()));
		operator.leftBumper()
			.onTrue(indexer.setTargetVelocityCommand(IndexerValue.OUTTAKE.getIndexerValue()))
			.onFalse(indexer.setTargetVelocityCommand(IndexerValue.STOP.getIndexerValue()));
		operator.rightBumper()
			.onTrue(kicker.setTargetVelocityCommand(KickerValue.OUTTAKE.getKickerValue()))
			.onFalse(kicker.setTargetVelocityCommand(KickerValue.STOP.getKickerValue()));
		operator.leftBumper()
			.onTrue(kicker.setTargetVelocityCommand(KickerValue.STOP.getKickerValue()));

	}

	public void resetClimb(){
		operator.rightBumper()
			.onTrue(new InstantCommand(()->climb.getMotor().setPower(0.25)))
			.onFalse(new InstantCommand(()->climb.getMotor().setPower(0)));

		operator.leftBumper()
			.onTrue(new InstantCommand(()->climb.getMotor().setPower(-0.25)))
			.onFalse(new InstantCommand(()->climb.getMotor().setPower(0)));
	}
}
