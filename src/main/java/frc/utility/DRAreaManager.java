package frc.utility;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;

public class DRAreaManager{
	private Rectangle2d redAllianceZone;
	private Rectangle2d blueAllianceZone;
	private Rectangle2d nuetralZone;

	public enum Zone {
		ALLIANCE_ZONE,
		OPPOSITION,
		NEUTRAL,
		BETWEEN
	}

	private Zone currentZone = Zone.ALLIANCE_ZONE;
	private SwerveDrive drive;

	public DRAreaManager(SwerveDrive drive) {
		this.drive = drive;
		redAllianceZone = new Rectangle2d(new Translation2d(12.5,0), new Translation2d(16.5, 8));
		blueAllianceZone = new Rectangle2d(new Translation2d(0,0), new Translation2d(4,8));
		nuetralZone = new Rectangle2d(new Translation2d(5, 0), new Translation2d(11,8));
	}

	public void periodic() {
		Translation2d drivePosition = drive.getTranslation2d();
		if (DroidRageConstants.alliance == Alliance.Red) {
			if (redAllianceZone.contains(drivePosition)) {
				currentZone = Zone.ALLIANCE_ZONE;
			} else if (blueAllianceZone.contains(drivePosition)) {
				currentZone = Zone.OPPOSITION;
			} else if (nuetralZone.contains(drivePosition)) {
				currentZone = Zone.NEUTRAL;
			} else 
				currentZone = Zone.BETWEEN;
		} else {
			if (redAllianceZone.contains(drivePosition)) {
				currentZone = Zone.OPPOSITION;
			} else if (blueAllianceZone.contains(drivePosition)) {
				currentZone = Zone.ALLIANCE_ZONE;
			} else if (nuetralZone.contains(drivePosition)) {
				currentZone = Zone.NEUTRAL;
			} else 
				currentZone = Zone.BETWEEN;
		}
		// SmartDashboard.putString("Current Zone", currentZone.toString());
	}

	public boolean isShootingArea() {
		return currentZone == Zone.ALLIANCE_ZONE;
	}

	public Zone getCurrentZone() {
		return currentZone;
	}
}