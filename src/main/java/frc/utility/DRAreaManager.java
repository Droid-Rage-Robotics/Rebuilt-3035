package frc.utility;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
	private double freeZoneStartY = 1.7, freeZoneEndY = 6.3;

	public DRAreaManager(SwerveDrive drive) {
		this.drive = drive;
		blueAllianceZone = new Rectangle2d(new Translation2d(0,0), new Translation2d(3.25,8));
		nuetralZone = new Rectangle2d(new Translation2d(5.75, 0), new Translation2d(10.25,8));
		redAllianceZone = new Rectangle2d(new Translation2d(12.75,0), new Translation2d(16.5, 8));

		updateZone();
		
		// blueAllianceZone = new Rectangle2d(new Translation2d(0,0), new Translation2d(3,8));
		// nuetralZone = new Rectangle2d(new Translation2d(6, 0), new Translation2d(10,8));
		// redAllianceZone = new Rectangle2d(new Translation2d(13,0), new Translation2d(16.5, 8));


		TelemetryUtils.registerDashboard(this);
	}

	public void periodic() {
		updateZone();
	}

	public void updateZone() {
		Translation2d drivePosition = drive.getState().Pose.getTranslation();
		if (DroidRageConstants.alliance == Alliance.Red) {
			if (redAllianceZone.contains(drivePosition)) {
				currentZone = Zone.ALLIANCE_ZONE;
			} else if (blueAllianceZone.contains(drivePosition)) {
				currentZone = Zone.OPPOSITION;
			} else if (nuetralZone.contains(drivePosition)) {
				currentZone = Zone.NEUTRAL;
			} else 
				if (drivePosition.getY() < freeZoneStartY || freeZoneEndY < drivePosition.getY()){
					if (DriverStation.isAutonomous() && drivePosition.getX()>12.5){
						currentZone = Zone.ALLIANCE_ZONE;
					}
					else{
						currentZone = Zone.BETWEEN;
					}
				} else {
					currentZone = Zone.NEUTRAL;
				}
		} else {
			if (redAllianceZone.contains(drivePosition)) {
				currentZone = Zone.OPPOSITION;
			} else if (blueAllianceZone.contains(drivePosition)) {
				currentZone = Zone.ALLIANCE_ZONE;
			} else if (nuetralZone.contains(drivePosition)) {
				currentZone = Zone.NEUTRAL;
			} else 
				if (drivePosition.getY() < freeZoneStartY || freeZoneEndY < drivePosition.getY()){
					if (DriverStation.isAutonomous() && drivePosition.getX()<4){
						currentZone = Zone.ALLIANCE_ZONE;
					}
					else{
						currentZone = Zone.BETWEEN;
					}
				} else {
					currentZone = Zone.NEUTRAL;
				}
		}
	}

	public boolean isShootingArea() {
		return currentZone == Zone.ALLIANCE_ZONE;
	}

	public static Zone getCurrentZone() {
		return currentZone;
	}

	// public static Trigger inAllianceZone() {
	// 	return new Trigger(() -> (currentZone==Zone.ALLIANCE_ZONE));
	// }

	// public static Trigger inOpposition() {
	// 	return new Trigger(() -> (currentZone==Zone.OPPOSITION));
	// }

	// public static Trigger inNeutral() {
	// 	return new Trigger(() -> (currentZone==Zone.NEUTRAL));
	// }

	// public static Trigger inBetween() {
	// 	return new Trigger(() -> (currentZone==Zone.BETWEEN));
	// }

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