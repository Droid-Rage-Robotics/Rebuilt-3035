// package frc.robot.commands.drive;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.DroidRageConstants;
// import frc.robot.subsystems.carriage.Carriage;
// import frc.robot.subsystems.drive.SwerveDrive;
// import frc.robot.subsystems.vision.Vision;

// public class TeleopAlignLevel extends Command{
// 	private SwerveDrive drive;
// 	private Vision vision;
// 	private CommandXboxController driver;
// 	private Carriage carriage;

// 	//TELEOP
// 	public TeleopAlignLevel(SwerveDrive drive, Carriage carriage, Vision vision, CommandXboxController driver) {
// 		this.driver = driver;
// 		this.carriage = carriage;
// 		this.drive = drive;
// 		this.vision = vision;
// 		addRequirements(drive, vision);
// 	}

// 	@Override
// 	public void execute() {
// 		switch (DroidRageConstants.alignmentMode) {
// 			case LEFT:
// 				if(!vision.gettV(DroidRageConstants.leftLimelight)){
// 					return;
// 				}
// 				break;
// 			case RIGHT:
// 				if (!vision.gettV(DroidRageConstants.rightLimelight)) {
// 					return;
// 				}
// 				break;
// 		}
// 		switch(carriage.getPosition()){
// 			case L4:
// 				drive.drive(range(Vision.Location.LEFT_L_L4, Vision.Location.RIGHT_R_L4), 0, 
// 					aim(Vision.Location.LEFT_L_L4, Vision.Location.RIGHT_R_L4)-0.03);
// 				break;
// 			case L3:
// 				drive.drive(range(Vision.Location.LEFT_L_L4, Vision.Location.RIGHT_R_L3), 0, 
// 					aim(Vision.Location.LEFT_L_L4, Vision.Location.RIGHT_R_L3)-0.03);
// 				break;
// 			case L2:
// 				drive.drive(range(Vision.Location.LEFT_L_L4, Vision.Location.RIGHT_R_L2), 0, 
// 					aim(Vision.Location.LEFT_L_L4, Vision.Location.RIGHT_R_L2)-0.03);
// 				break;
// 			default:
// 				return;
// 		}
// 	}
  
// 	@Override
// 	public boolean isFinished() {
// 		return !driver.leftBumper().getAsBoolean();
// 	}

// 	double aim(Vision.Location right, Vision.Location left) {
// 		double targetingAngularVelocity =0;
// 		switch(DroidRageConstants.alignmentMode){
// 			case LEFT:
// 				targetingAngularVelocity = vision.rotController.calculate(
// 					vision.gettX(DroidRageConstants.leftLimelight), left.getAngle());
// 				break;
// 			case RIGHT:
// 				targetingAngularVelocity = vision.rotController.calculate(
// 					vision.gettX(DroidRageConstants.rightLimelight), right.getAngle());
// 				break;
// 		}
// 		return targetingAngularVelocity;
// 	}

// 	double range(Vision.Location right, Vision.Location left) {
// 		double targetingForwardSpeed =0;
// 		switch(DroidRageConstants.alignmentMode){
// 			case LEFT:
// 				targetingForwardSpeed = vision.xController.calculate(
// 					vision.gettY(DroidRageConstants.leftLimelight), left.getDistance());
// 				break;
// 			case RIGHT:
// 				targetingForwardSpeed = vision.xController.calculate(
// 					vision.gettY(DroidRageConstants.rightLimelight), right.getDistance());
// 				break;
// 		}
		
// 		return targetingForwardSpeed;
// 	}

// }
