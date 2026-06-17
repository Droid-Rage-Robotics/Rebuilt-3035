package frc.robot.subsystems.drive.maple;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

/** PathPlanner holonomic controller with AdvantageKit-visible tracking telemetry. */
public class DRHolonomicSwerveController implements PathFollowingController {
    private static final double DEFAULT_PERIOD_SECONDS = 0.02;

    private final String logKey;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    private boolean enabled = true;

    public DRHolonomicSwerveController(
            PIDConstants translationConstants, PIDConstants rotationConstants, String logKey) {
        this(translationConstants, rotationConstants, logKey, DEFAULT_PERIOD_SECONDS);
    }

    public DRHolonomicSwerveController(
            PIDConstants translationConstants,
            PIDConstants rotationConstants,
            String logKey,
            double periodSeconds) {
        this.logKey = logKey;

        xController =
                new PIDController(
                        translationConstants.kP,
                        translationConstants.kI,
                        translationConstants.kD,
                        periodSeconds);
        xController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);

        yController =
                new PIDController(
                        translationConstants.kP,
                        translationConstants.kI,
                        translationConstants.kD,
                        periodSeconds);
        yController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);

        rotationController =
                new PIDController(
                        rotationConstants.kP,
                        rotationConstants.kI,
                        rotationConstants.kD,
                        periodSeconds);
        rotationController.setIntegratorRange(-rotationConstants.iZone, rotationConstants.iZone);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        xController.reset();
        yController.reset();
        rotationController.reset();

        Logger.recordOutput(logKey + "/ResetPose", currentPose);
        Logger.recordOutput(logKey + "/ResetSpeeds", currentSpeeds);
    }

    @Override
    public ChassisSpeeds calculateRobotRelativeSpeeds(
            Pose2d currentPose, PathPlannerTrajectoryState targetState) {
        double xFeedforward = targetState.fieldSpeeds.vxMetersPerSecond;
        double yFeedforward = targetState.fieldSpeeds.vyMetersPerSecond;
        double rotationFeedforward = targetState.fieldSpeeds.omegaRadiansPerSecond;

        double xFeedback = 0.0;
        double yFeedback = 0.0;
        double rotationFeedback = 0.0;

        if (enabled) {
            xFeedback = xController.calculate(currentPose.getX(), targetState.pose.getX());
            yFeedback = yController.calculate(currentPose.getY(), targetState.pose.getY());
            rotationFeedback =
                    rotationController.calculate(
                            currentPose.getRotation().getRadians(),
                            targetState.pose.getRotation().getRadians());
        }

        ChassisSpeeds output =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xFeedforward + xFeedback,
                        yFeedforward + yFeedback,
                        rotationFeedforward + rotationFeedback,
                        currentPose.getRotation());

        Translation2d translationError =
                currentPose.getTranslation().minus(targetState.pose.getTranslation());

        Logger.recordOutput(logKey + "/CurrentPose", currentPose);
        Logger.recordOutput(logKey + "/TargetPose", targetState.pose);
        Logger.recordOutput(logKey + "/XFeedforwardMetersPerSec", xFeedforward);
        Logger.recordOutput(logKey + "/YFeedforwardMetersPerSec", yFeedforward);
        Logger.recordOutput(logKey + "/RotationFeedforwardRadPerSec", rotationFeedforward);
        Logger.recordOutput(logKey + "/XFeedbackMetersPerSec", xFeedback);
        Logger.recordOutput(logKey + "/YFeedbackMetersPerSec", yFeedback);
        Logger.recordOutput(logKey + "/RotationFeedbackRadPerSec", rotationFeedback);
        Logger.recordOutput(logKey + "/TranslationErrorMeters", translationError.getNorm());
        Logger.recordOutput(
                logKey + "/RotationErrorDegrees",
                currentPose.getRotation().minus(targetState.pose.getRotation()).getDegrees());
        Logger.recordOutput(logKey + "/OutputSpeeds", output);

        return output;
    }

    @Override
    public boolean isHolonomic() {
        return true;
    }
}
