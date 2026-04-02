package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.Shooter;

import java.util.List;
import java.util.function.Supplier;


/**
 * Largely written by Eeshwar based off their blog at https://blog.eeshwark.com/robotblog/shooting-on-the-fly
 */
public class DRShooterTwo extends Command{

  // Subsystems
  private final Shooter shooter;
  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds;
  private final Pose2d goalPose;

  // Tuned Constants
  double totalExitVelocity = 15.0; // m/s
  /**
   * Time in seconds between when the robot is told to move and when the shooter actually shoots.
   */
  private final double latency      = 0.15;
  /**
   * Maps Distance to RPM
   */
  private static final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  	static{
        shooterTable.put(1.25,40.0);
        shooterTable.put(1.89,42.0);
        shooterTable.put(3.05,47.0);
        shooterTable.put(4.27,50.2);

		hoodMap.put(1.25,.0);
		hoodMap.put(1.89,11.2);
		hoodMap.put(3.05,12.1);
		hoodMap.put(4.27,12.75);
    }
  public DRShooterTwo(Shooter shooter, SwerveDrive drive,
							   Supplier<Pose2d> currentPose, Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
							   Pose2d goal){
	this.shooter = shooter;
	robotPose = ()->drive.getState().Pose;
	this.fieldOrientedChassisSpeeds = fieldOrientedChassisSpeeds;
	this.goalPose = goal;
  }

  @Override
  public void initialize()
  {

  }

  @Override
  public void execute()
  {
	// Please look here for the original authors work!
	// https://blog.eeshwark.com/robotblog/shooting-on-the-fly
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// YASS did not come up with this
	// -------------------------------------------------------

	var robotSpeed = fieldOrientedChassisSpeeds.get();
	// 1. LATENCY COMP
	Translation2d futurePos = robotPose.get().getTranslation().plus(
		new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
																   );

	// 2. GET TARGET VECTOR
	Translation2d goalLocation = goalPose.getTranslation();
	Translation2d targetVec    = goalLocation.minus(futurePos);
	double        dist         = targetVec.getNorm();

	// 3. CALCULATE IDEAL SHOT (Stationary)
	// Note: This returns HORIZONTAL velocity component
	double idealHorizontalSpeed = shooterTable.get(dist);

	// 4. VECTOR SUBTRACTION
	Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
	Translation2d shotVec     = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

	// 5. CONVERT TO CONTROLS
	double turretAngle        = shotVec.getAngle().getDegrees();
	double newHorizontalSpeed = shotVec.getNorm();

	// 6. SOLVE FOR NEW PITCH/RPM
	// Assuming constant total exit velocity, variable hood:
	// Clamp to avoid domain errors if we need more speed than possible
	double ratio    = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
	double newPitch = Math.acos(ratio);

	// 7. SET OUTPUTS
	shooter.getTurret().setGoalAngle(Degrees.of(turretAngle));
	// shooter.setRPM(calcRPM(totalExitVelocity));
	shooter.getHood().setGoalAngle(Rotation2d.fromRadians(newPitch));
	shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(totalExitVelocity));
  }

  @Override
  public boolean isFinished()
  {
	// TODO: Make this return true when this Command no longer needs to run execute()
	return false;
  }

  @Override
  public void end(boolean interrupted)
  {

  }
}