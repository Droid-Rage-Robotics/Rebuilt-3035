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
import frc.robot.commands.TeleopCommands;
import frc.robot.commands.Turn180Degrees;
import frc.robot.commands.SysId.SysIdCommand;
import frc.robot.commands.SysId.SysIdRoutineCommand;
import frc.robot.commands.drive.TeleopAlign;
import frc.robot.commands.manual.ManualClimb;
import frc.robot.commands.manual.ManualElevator;
import frc.robot.commands.manual.SwerveDriveTeleop;
import frc.robot.commands.manual.Turning;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorValue;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.Carriage.CarriageIntakeValue;
import frc.robot.subsystems.carriage.Carriage.CarriageValue;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
	private final CommandXboxController driver, operator;
	
	public RobotContainer(CommandXboxController driver, CommandXboxController operator){
		DriverStation.silenceJoystickConnectionWarning(true);
		this.driver = driver;
		this.operator = operator;

	}

	// public void configureTeleOpBindings(
	// 	SwerveDrive drive,
	// 	Elevator elevator,
	// 	Carriage carriage,
	// 	// Climb climb,
	// 	Vision vision
	// 	) {

	// 	SmartDashboard.putData("TeleopCommands/Barge", new TeleopCommands().barge(elevator, carriage));
	// 	SmartDashboard.putData("TeleopCommands/L4", new TeleopCommands().goL4(elevator, carriage));
	// 	SmartDashboard.putData("TeleopCommands/Reset Pos", new TeleopCommands().resetHP(elevator, carriage, CarriageValue.INTAKE_HPS));
	// 	SmartDashboard.putData("TeleopCommands/L3", new SequentialCommandGroup(
	// 		carriage.setPositionCommand(CarriageValue.L3),
	// 		elevator.setTargetPositionCommand(ElevatorValue.L3)));
	// 	SmartDashboard.putData("TeleopCommands/L2", new SequentialCommandGroup(
	// 		carriage.setPositionCommand(CarriageValue.L2),
	// 		elevator.setTargetPositionCommand(ElevatorValue.L2)));
	// 	SmartDashboard.putData("TeleopCommands/L1", new SequentialCommandGroup(
	// 		carriage.setPositionCommand(CarriageValue.L1),
	// 		elevator.setTargetPositionCommand(ElevatorValue.L1)));

	// 	SmartDashboard.putData("TeleopCommands/Algae/Algae Ground", new SequentialCommandGroup(
	// 		carriage.setPositionCommand(CarriageValue.INTAKE_GROUND),
	// 		elevator.setTargetPositionCommand(ElevatorValue.GROUND)
	// 	));

	// 	SmartDashboard.putData("TeleopCommands/Algae/Algae Low", new SequentialCommandGroup(
	// 		carriage.setPositionCommand(CarriageValue.ALGAE_LOW),
	// 		elevator.setTargetPositionCommand(ElevatorValue.ALGAE_LOW)
	// 	));

	// 	SmartDashboard.putData("TeleopCommands/Algae/Algae High", new SequentialCommandGroup(
	// 		carriage.setPositionCommand(CarriageValue.ALGAE_HIGH),
	// 		elevator.setTargetPositionCommand(ElevatorValue.ALGAE_HIGH)
	// 	));

		


		
	// 	// driver.a().onTrue(new Turn180Degrees(drive, driver)); //ToDo: Test

	// 	// RumbleCommand rum = new RumbleCommand(driver);

	// 	// driver.b().whileTrue(rum);

	// 	// Slow Mode and Gyro Reset in the Default Command
	// 	drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver, elevator));
	// 	// drive.setDefaultCommand(new Turning(drive, driver, elevator));
	// 	elevator.setDefaultCommand(new ManualElevator(elevator, operator::getRightY));
	// 	// vision.setDefaultCommand(new LightCommand(driver, vision));
	// 	// vision.setDefaultCommand(new RumbleCommand(elevator, carriage, driver, operator, vision));
	// 	// climb.setDefaultCommand(new ManualClimb(climb, operator::getLeftY));

	// 	driver.leftBumper()
	// 		.onTrue(new TeleopAlign(drive, vision, driver));
	// 	driver.rightStick()
	// 		.onTrue(new InstantCommand(() -> DroidRageConstants.setAlignment((DroidRageConstants.Alignment.RIGHT))));
	// 	driver.leftStick()
	// 		.onTrue(new InstantCommand(() -> DroidRageConstants.setAlignment((DroidRageConstants.Alignment.LEFT))));
	// 	driver.rightTrigger() ///TODO: Test on False
	// 		.onTrue(carriage.setIntakeCommand(CarriageIntakeValue.INTAKE))
	// 		.onFalse(new TeleopCommands().teleopHoldCommand(carriage));
	// 	driver.leftTrigger()
	// 		.onTrue(new TeleopCommands().teleopOuttakeCommand(carriage))
	// 		.onFalse(carriage.setIntakeCommand(CarriageIntakeValue.STOP));

	// 	driver.x() //To Test
	// 		.onTrue(new TeleopCommands().barge(elevator, carriage));

	// 	// driver.povUp()
	// 	// 	.onTrue(climb.setTargetPositionCommand(Climb.hold));
	// 	// driver.povDown()
	// 	// 	.onTrue(climb.setTargetPositionCommand(Climb.climb));

	// 	// driver.povRight()
	// 	// 	.onTrue(new TeleopCommands().resetHP(elevator, carriage, CarriageValue.BARGE_HOLD));
	// 	operator.y()
	// 		.onTrue(new TeleopCommands().goL4(elevator, carriage));
	// 	operator.x()
	// 		.onTrue(
	// 			new SequentialCommandGroup(
	// 				carriage.setPositionCommand(CarriageValue.L3),
	// 				elevator.setTargetPositionCommand(ElevatorValue.L3)));
	// 	operator.b()
	// 		.onTrue(
	// 			new ParallelCommandGroup(
	// 				carriage.setPositionCommand(CarriageValue.L2),
	// 				elevator.setTargetPositionCommand(ElevatorValue.L2)
	// 			)
	// 		);
	// 	operator.a()
	// 		.onTrue(
	// 			new ParallelCommandGroup(
	// 				carriage.setPositionCommand(CarriageValue.L1),
	// 				elevator.setTargetPositionCommand(ElevatorValue.L1)
	// 			)
	// 		);
			
		

	// 	operator.povRight()// Algae
	// 		.onTrue(carriage.setPositionCommand(CarriageValue.PROCESSOR))
	// 		.onTrue(elevator.setTargetPositionCommand(ElevatorValue.PROCESSOR));
	// 	operator.povLeft()// Coral
	// 		.onTrue(new TeleopCommands().resetHP(elevator, carriage, CarriageValue.INTAKE_HPS_BLOCK));
	// 	operator.povUp()// ALgae
	// 		.onTrue(
	// 			new SequentialCommandGroup(
	// 				carriage.setPositionCommand(CarriageValue.INTAKE_GROUND),
	// 				elevator.setTargetPositionCommand(ElevatorValue.GROUND)
	// 			)
	// 		);
	// 	operator.povDown()
	// 		.onTrue(new TeleopCommands().resetHP(elevator, carriage, CarriageValue.INTAKE_HPS));

	// 	operator.rightBumper()
	// 		.onTrue(carriage.setPositionCommand(CarriageValue.ALGAE_HIGH))
	// 		.onTrue(elevator.setTargetPositionCommand(ElevatorValue.ALGAE_HIGH));
	// 	operator.leftBumper()
	// 		.onTrue(carriage.setPositionCommand(CarriageValue.ALGAE_LOW))
	// 		.onTrue(elevator.setTargetPositionCommand(ElevatorValue.ALGAE_LOW));
		


	// 	// rumble.onTrue(
    //     // new StartEndCommand(
    //     //         () -> {
    //     //           driver.setRumble(RumbleType.kLeftRumble, 1.0);
    //     //           driver.setRumble(RumbleType.kRightRumble, 1.0);
    //     //         },
    //     //         () -> {
    //     //           driver.setRumble(RumbleType.kLeftRumble, 0.0);
    //     //           driver.setRumble(RumbleType.kRightRumble, 0.0);
    //     //         })
    //     //     .withTimeout(0.5));
	// }

	public void testDrive(CommandXboxController driver, SwerveDrive drive, Vision vision) {
		drive.setDefaultCommand(new SwerveDriveTeleop(drive, driver));
		// drive.setDefaultCommand(new Turning(drive, driver));

		// 	driver.leftBumper()
	// 		.onTrue(new TeleopAlign(drive, vision, driver));
	
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
	
	public void sysID(CommandXboxController driver, SysIdRoutine routine, Carriage carriage) {
        driver.b().onTrue(carriage.setPositionCommand(CarriageValue.L2));
		driver.a().onTrue(new SysIdRoutineCommand(routine));
    }
}
