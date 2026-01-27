package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class AutoAlign extends Command {
	private final SwerveDrive drive;
	private final Vision vision;
	// private Timer timer= new Timer();
	
    // private static final ShuffleboardValue<Double> aim = 
    //     ShuffleboardValue.create(0.0, "Aim", Vision.class.getSimpleName()).build();
		
	// 	private static final ShuffleboardValue<Double> range = 
    //     ShuffleboardValue.create(0.0, "Range", Vision.class.getSimpleName()).build();
	private final Supplier<Double> aim, range;
	private volatile double xSpeed, turnSpeed;
	

	// Auto
	public AutoAlign(SwerveDrive drive, Vision vision) {
		this.drive = drive;
		this.vision = vision;
		this.aim = vision::aim;
		this.range = vision::range;
		addRequirements(vision);
	}

	@Override
	public void execute() {
		xSpeed = range.get();
		turnSpeed = aim.get();
		
		switch (DroidRageConstants.alignmentMode) {
			case LEFT:
				if (!vision.getTV(DroidRageConstants.leftLimelight)) {
					return;
				}
				break;
			case RIGHT:
				if (!vision.getTV(DroidRageConstants.turretLL)) {
					return;
				}
				break;
			case MIDDLE:
				if (!vision.getTV(DroidRageConstants.turretLL) &&
						(!vision.getTV(DroidRageConstants.leftLimelight))) {
					return;
				}
				break;
		}

		// if (!vision.gettV(DroidRageConstants.rightLimelight) &&
		// 		(!vision.gettV(DroidRageConstants.leftLimelight))) {
		// 	return;
		// }
		// aim.set((vision.aim()));
		// range.set(vision.range());
		// drive.drive(vision.range(), 0, vision.aim());
		drive.drive(xSpeed, 0,turnSpeed);
		vision.isAlign.set(true);
	}

	@Override
	public boolean isFinished() {
		// return !driver.povUp().getAsBoolean();
		// if(vision.rotController.atSetpoint() && vision.xController.atSetpoint()){
		// 	vision.isAlignWriter.set(false);
		// 	return true;
		// }
		return (vision.rotController.atSetpoint() && vision.xController.atSetpoint());// || timer.hasElapsed(5);
	}

}
