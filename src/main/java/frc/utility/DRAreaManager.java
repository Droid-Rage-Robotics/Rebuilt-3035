package frc.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;

public class DRAreaManager {
	private Rectangle2d redAllianceZone;
	private Rectangle2d blueAllianceZone;
	private Rectangle2d nuetralZone;

	public enum Zone {
		ALLIANCE_ZONE,
		OPPOSITION,
		NEUTRAL
	}

	private Zone currentZone = Zone.ALLIANCE_ZONE;
	private SwerveDrive drive;

	public DRAreaManager(SwerveDrive drive) {
		this.drive = drive;
		redAllianceZone = new Rectangle2d(new Translation2d(0, 0), new Translation2d(0, 0));
		blueAllianceZone = new Rectangle2d(new Translation2d(16.4592, 0), new Translation2d(24.6888, 16.4592));
		nuetralZone = new Rectangle2d(new Translation2d(0, 0), new Translation2d(0, 0));
	}

	public void periodic() {
		Translation2d drivePosition = drive.getTranslation2d();
		drive.getRotation3d();
		if (DroidRageConstants.alliance == Alliance.Red) {
		if (redAllianceZone.contains(drivePosition)) {
			currentZone = Zone.ALLIANCE_ZONE;
		} else if (blueAllianceZone.contains(drivePosition)) {
			currentZone = Zone.OPPOSITION;
		} else if (nuetralZone.contains(drivePosition)) {
			currentZone = Zone.NEUTRAL;
		}
		} else {
		if (redAllianceZone.contains(drivePosition)) {
			currentZone = Zone.OPPOSITION;
		} else if (blueAllianceZone.contains(drivePosition)) {
			currentZone = Zone.ALLIANCE_ZONE;
		} else if (nuetralZone.contains(drivePosition)) {
			currentZone = Zone.NEUTRAL;
		}
		}
	}

	public boolean isShootingArea() {
		return currentZone == Zone.ALLIANCE_ZONE;
	}

	public Zone getCurrentZone() {
		return currentZone;
	}
}