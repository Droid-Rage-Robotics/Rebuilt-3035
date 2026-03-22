package frc.utility;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.utility.TelemetryUtils.Dashboard;

public class DRAreaManager implements Dashboard, Sendable{
	public enum Zone {
		ALLIANCE_ZONE,
		OPPOSITION,
		NEUTRAL,
		BETWEEN
	}

	private final SwerveDrive drive;
	private final Rectangle2d redAllianceZone;
	private final Rectangle2d blueAllianceZone;
	private final Rectangle2d nuetralZone;

	private static Zone currentZone = Zone.ALLIANCE_ZONE;

	public DRAreaManager(SwerveDrive drive) {
		this.drive = drive;
		redAllianceZone = new Rectangle2d(new Translation2d(12.5,0), new Translation2d(16.5, 8));
		blueAllianceZone = new Rectangle2d(new Translation2d(0,0), new Translation2d(4,8));
		nuetralZone = new Rectangle2d(new Translation2d(5, 0), new Translation2d(11,8));
	}

	public void periodic() {
		Translation2d drivePosition = drive.getState().Pose.getTranslation();
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
	}

	public boolean isShootingArea() {
		return currentZone == Zone.ALLIANCE_ZONE;
	}

	public static Zone getCurrentZone() {
		return currentZone;
	}

	public static Trigger inAllianceZone() {
		return new Trigger(() -> (currentZone==Zone.ALLIANCE_ZONE));
	}

	public static Trigger inOpposition() {
		return new Trigger(() -> (currentZone==Zone.OPPOSITION));
	}

	public static Trigger inNeutral() {
		return new Trigger(() -> (currentZone==Zone.NEUTRAL));
	}

	public static Trigger inBetween() {
		return new Trigger(() -> (currentZone==Zone.BETWEEN));
	}

	@Override
	public void elasticInit() {
		SmartDashboard.putData("Drive/AreaManager", this );
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addStringProperty("CurrentZone", () -> getCurrentZone().toString(), null);
	}

	@Override
	public void practiceWriters() {}

	@Override
	public void alerts() {}
}