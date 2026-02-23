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
import frc.robot.commands.LightCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.ClimbValue;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerValue;
import frc.robot.subsystems.Light;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.robot.subsystems.shooter.Kicker;
import frc.robot.subsystems.shooter.Kicker.KickerValue;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
import frc.robot.subsystems.vision.Vision;
import frc.utility.ControllerUtils;

public class RobotContainer {
	private double MaxSpeed = 0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.4).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // /* Setting up bindings for necessary control of the swerve drive platform */
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         // .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
	
	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

	
	private final CommandXboxController driver, operator;
	
	public RobotContainer(CommandXboxController driver, CommandXboxController operator){
		DriverStation.silenceJoystickConnectionWarning(true);
		this.driver = driver;
		this.operator = operator;

	}

	public void configureTeleOpBindings(
		// SwerveDrive drive,
		Intake intake,
		Climb climb,
		Indexer indexer,
		Kicker kicker,
		Shooter shooter,
		Vision vision,
		Light light
		) {
			// Slow Mode and Gyro Reset in the Default Command
			// drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
			// drive.setDefaultCommand(new Turning(drive, driver));
			// climb.setDefaultCommand(new ClimbTeleop(climb, operator::getRightY));
			// climb.setDefaultCommand(new ManualClimb(climb, operator::getLeftY));

			// light.setDefaultCommand(new LightCommand(light));
			// driver.rightTrigger()
			// 	.onTrue(intake.setPositionCommand(IntakeValue.INTAKE))
			// 	.onFalse(intake.setPositionCommand(IntakeValue.STOP));
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
    			
			driver.rightBumper()
    			.onTrue(indexer.setTargetVelocityCommand(IndexerValue.INTAKE.getIndexerValue()))
   				.onFalse(indexer.setTargetVelocityCommand(IndexerValue.STOP.getIndexerValue()));

			driver.leftBumper()
    			.onTrue(indexer.setTargetVelocityCommand(IndexerValue.OUTTAKE.getIndexerValue()))
   				.onFalse(indexer.setTargetVelocityCommand(IndexerValue.STOP.getIndexerValue()));

			driver.rightBumper()
				.onTrue(kicker.setTargetVelocityCommand(KickerValue.OUTTAKE.getKickerValue()))
				.onFalse(kicker.setTargetVelocityCommand(KickerValue.STOP.getKickerValue()));

			driver.rightBumper()
				.onTrue(shooter.getShooter().setTargetVelocityCommand(-25))
				.onFalse(shooter.getShooter().setTargetVelocityCommand(0));

			// driver.b().onTrue(shooter.getTurret().setTargetPositionCommand(-90))
			//     .onFalse(shooter.getTurret(  ).setTargetPositionCommand(0));

			driver.a().onTrue(shooter.getHood().setTargetPositionCommand(10))
			    .onFalse(shooter.getHood().setTargetPositionCommand(0));
			
			// shooter.getTurret().setGoalAngle(Rotation2d.fromRotations(ControllerUtils.getRightStickDeg(driver)/360.0));

			driver.x().onTrue(intake.getPivot().setTargetPositionCommand(IntakeValue.INTAKE.getPivotAngle()));
			    // .onFalse(intake.getPivot().setTargetPositionCommand(IntakeValue.STOP.getPivotAngle()));

			driver.povDown().onTrue(intake.getPivot().setTargetPositionCommand(IntakeValue.STOP.getPivotAngle()));


			// operator.a()
    		// 	.onTrue(shooter.setShooterModeCommand(ShooterMode.HOLD)); //LED strip: indicate the mode, one automatic (automates itself), three positions
			// operator.b()
    		// 	.onTrue(shooter.setShooterModeCommand(ShooterMode.OPPOSITE));
			// operator.x()
			// 	.onTrue(shooter.setShooterModeCommand(ShooterMode.SCORE));
			// operator.y()
			// 	.onTrue(shooter.setShooterModeCommand(ShooterMode.HOARD));
				
	}

	// public void testDrive(CommandXboxController driver, SwerveDrive drive, Vision vision) {
	// 	drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
	// }
}
