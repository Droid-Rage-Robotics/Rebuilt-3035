package frc.robot;


import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.LightCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.manual.SwerveDriveTeleop;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.ClimbValue;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerValue;
import frc.robot.subsystems.Light;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveConfig;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.intake.IntakeWheel;
import frc.robot.subsystems.intake.Pivot;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Kicker;
import frc.robot.subsystems.shooter.Kicker.KickerValue;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
import frc.robot.subsystems.shooter.ShooterWheel;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
	private final SwerveConfig swerveConfig = new SwerveConfig();
	public final SwerveDrive drive = new SwerveDrive(true,swerveConfig);
	private final Vision vision = new Vision();
    private final Climb climb = new Climb(false);
    
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

    private final Light light = new Light(0);
    
    private final CommandXboxController driver =
		new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
	
	private final CommandXboxController operator =		
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);
	
    private double MaxAngularRate = RotationsPerSecond.of(0.4).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry();
	
	

	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);
	}

	public void configureTeleOpBindings() {
		drive.registerTelemetry(logger::telemeterize);

		// Slow Mode and Gyro Reset in the Default Command
		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
		// drive.setDefaultCommand(new Turning(drive, driver));
		// climb.setDefaultCommand(new ClimbTeleop(climb, operator::getRightY));
		// climb.setDefaultCommand(new ManualClimb(climb, operator::getLeftY));

		// light.setDefaultCommand(new LightCommand(light));
		operator.rightTrigger()
			.onTrue(intake.setPositionCommand(IntakeValue.INTAKE))
			.onFalse(intake.setPositionCommand(IntakeValue.STOP));
			// .onFalse(new ParallelCommandGroup(
			// 	intake.getPivot().setTargetPositionCommand(IntakeValue.OUTTAKE.getPivotAngle()),
			// 	new SequentialCommandGroup(
			// 		intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.STOP.getIntakeSpeed()),
			// 		new WaitCommand(0.75),
			// 		intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.OUTTAKE.getIntakeSpeed()),
			// 		new WaitCommand(1),
			// 		new ParallelCommandGroup(
			// 			intake.getPivot().setTargetPositionCommand(IntakeValue.INTAKE.getPivotAngle()),
			// 			intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.STOP.getIntakeSpeed())
			// 		)
			// 	)
				

			// ));
			// .onFalse(intake.setPositionCommand(IntakeValue.OUTTAKE));
		driver.leftTrigger()
			.onTrue(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.OUTTAKE.getIntakeSpeed()))
			.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.STOP.getIntakeSpeed()));

		driver.rightTrigger()
			.onTrue(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.INTAKE.getIntakeSpeed()))
			.onFalse(intake.getIntakeWheel().setTargetVelocityCommand(IntakeValue.STOP.getIntakeSpeed()));


		// operator.rightTrigger()
		// 	.onTrue(climb.setTargetPositionCommand(ClimbValue.CLIMB.getHeight()))
		// 	.onFalse(climb.setTargetPositionCommand(ClimbValue.START.getHeight()));
			
		operator.rightBumper()
			.onTrue(indexer.setTargetVelocityCommand(IndexerValue.INTAKE.getIndexerValue()))
			.onFalse(indexer.setTargetVelocityCommand(IndexerValue.STOP.getIndexerValue()));

		operator.leftBumper()
			.onTrue(indexer.setTargetVelocityCommand(IndexerValue.OUTTAKE.getIndexerValue()))
			.onFalse(indexer.setTargetVelocityCommand(IndexerValue.STOP.getIndexerValue()));

		operator.rightBumper()
			.onTrue(kicker.setTargetVelocityCommand(KickerValue.OUTTAKE.getKickerValue()))
			.onFalse(kicker.setTargetVelocityCommand(KickerValue.STOP.getKickerValue()));

		operator.rightBumper()
			.onTrue(shooter.getShooter().setTargetVelocityCommand(-25))
			.onFalse(shooter.getShooter().setTargetVelocityCommand(0));

		// driver.b().onTrue(shooter.getTurret().setTargetPositionCommand(-90))
		//     .onFalse(shooter.getTurret(  ).setTargetPositionCommand(0));

		operator.a().onTrue(shooter.getHood().setTargetPositionCommand(10))
			.onFalse(shooter.getHood().setTargetPositionCommand(0));
		
		// shooter.getTurret().setGoalAngle(Rotation2d.fromRotations(ControllerUtils.getRightStickDeg(driver)/360.0));

		operator.x().onTrue(intake.getPivot().setTargetPositionCommand(IntakeValue.INTAKE.getPivotAngle()));
			// .onFalse(intake.getPivot().setTargetPositionCommand(IntakeValue.STOP.getPivotAngle()));

		operator.povDown().onTrue(intake.getPivot().setTargetPositionCommand(IntakeValue.STOP.getPivotAngle()));


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
		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));

		driver.back().and(driver.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drive.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
	}
}
