package frc.robot.commands.autos.pathplanner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

import org.littletonrobotics.junction.Logger;

/**
 * Follows a PathPlanner path and repairs large tracking errors by cancelling the active follower and
 * pathfinding back to the path.
 */
public class AutoCorrectingPath extends Command {
    private static final String LOG_KEY = "AutoCorrectingPath";

    private final Drive drive;
    private final PathPlannerPath path;
    private final PathConstraints pathfindingConstraints;
    private final RobotConfig robotConfig;
    private final boolean resetOdom;
    private final boolean pathfindAtStart;
    private final double replanThresholdMeters;
    private final double replanCooldownSeconds;
    private final double minimumReplanTimeSeconds;

    private final Timer pathTimer = new Timer();
    private final Timer replanCooldownTimer = new Timer();

    private Command activeCommand = Commands.none();
    private PathPlannerTrajectory referenceTrajectory;
    private int replanCount = 0;

    private AutoCorrectingPath(
            Drive drive,
            PathPlannerPath path,
            PathConstraints pathfindingConstraints,
            boolean resetOdom,
            boolean pathfindAtStart,
            double replanThresholdMeters,
            double replanCooldownSeconds,
            double minimumReplanTimeSeconds) {
        this.drive = drive;
        this.path = path;
        this.pathfindingConstraints = pathfindingConstraints;
        this.robotConfig = drive.getPathPlannerRobotConfig();
        this.resetOdom = resetOdom;
        this.pathfindAtStart = pathfindAtStart;
        this.replanThresholdMeters = replanThresholdMeters;
        this.replanCooldownSeconds = replanCooldownSeconds;
        this.minimumReplanTimeSeconds = minimumReplanTimeSeconds;
    }

    public static Builder named(Drive drive, String pathName) {
        return new Builder(drive, PathSource.PATHPLANNER, pathName);
    }

    public static Builder choreo(Drive drive, String trajectoryName) {
        return new Builder(drive, PathSource.CHOREO, trajectoryName);
    }

    public static Builder choreo(Drive drive, String trajectoryName, int splitIndex) {
        return new Builder(drive, trajectoryName, splitIndex);
    }

    public static Builder path(Drive drive, PathPlannerPath path) {
        return new Builder(drive, path);
    }

    @Override
    public void initialize() {
        replanCount = 0;
        referenceTrajectory = buildReferenceTrajectory();

        pathTimer.restart();
        replanCooldownTimer.restart();

        scheduleInitialCommand();
        logState("Initialized", 0.0, Pose2d.kZero);
    }

    @Override
    public void execute() {
        if (referenceTrajectory == null || activeCommand == null) {
            return;
        }

        double elapsed = pathTimer.get();
        Pose2d currentPose = drive.getPose();
        Pose2d referencePose =
                referenceTrajectory.sample(
                                Math.min(elapsed, referenceTrajectory.getTotalTimeSeconds()))
                        .pose;
        double errorMeters =
                currentPose.getTranslation().getDistance(referencePose.getTranslation());

        logState("Following", errorMeters, referencePose);

        boolean canReplan =
                elapsed >= minimumReplanTimeSeconds
                        && replanCooldownTimer.hasElapsed(replanCooldownSeconds)
                        && activeCommand.isScheduled();

        if (errorMeters > replanThresholdMeters && canReplan) {
            replanCount++;
            DriverStation.reportWarning(
                    "[AutoCorrectingPath] Replanning "
                            + path.name
                            + " after "
                            + String.format("%.2f", errorMeters)
                            + " m error",
                    false);

            activeCommand.cancel();
            activeCommand = AutoBuilder.pathfindThenFollowPath(path, pathfindingConstraints);
            CommandScheduler.getInstance().schedule(activeCommand);
            replanCooldownTimer.restart();
            logState("Replanned", errorMeters, referencePose);
        }
    }

    @Override
    public boolean isFinished() {
        return activeCommand != null && !activeCommand.isScheduled();
    }

    @Override
    public void end(boolean interrupted) {
        pathTimer.stop();
        replanCooldownTimer.stop();

        if (activeCommand != null && activeCommand.isScheduled()) {
            activeCommand.cancel();
        }

        Logger.recordOutput(LOG_KEY + "/State", interrupted ? "Interrupted" : "Finished");
        Logger.recordOutput(LOG_KEY + "/Active", false);
        Logger.recordOutput(LOG_KEY + "/ReplanCount", replanCount);
    }

    private void scheduleInitialCommand() {
        if (pathfindAtStart) {
            activeCommand = AutoBuilder.pathfindThenFollowPath(path, pathfindingConstraints);
        } else if (resetOdom) {
            activeCommand = AutoBuilder.followPath(path);
            if (path.getStartingHolonomicPose().isPresent()) {
                activeCommand =
                        AutoBuilder.resetOdom(path.getStartingHolonomicPose().get())
                                .andThen(activeCommand);
            }
        } else {
            activeCommand = AutoBuilder.followPath(path);
        }

        CommandScheduler.getInstance().schedule(activeCommand);
    }

    private PathPlannerTrajectory buildReferenceTrajectory() {
        PathPlannerPath selectedReferencePath = path;
        if (AutoBuilder.shouldFlip() && !path.preventFlipping) {
            selectedReferencePath = path.flipPath();
        }

        PathPlannerPath referencePath = selectedReferencePath;
        return referencePath
                .getIdealTrajectory(robotConfig)
                .orElseGet(
                        () ->
                                referencePath.generateTrajectory(
                                        drive.getChassisSpeeds(), drive.getRotation(), robotConfig));
    }

    private void logState(String state, double errorMeters, Pose2d referencePose) {
        Logger.recordOutput(LOG_KEY + "/State", state);
        Logger.recordOutput(LOG_KEY + "/Active", activeCommand != null && activeCommand.isScheduled());
        Logger.recordOutput(LOG_KEY + "/PathName", path.name);
        Logger.recordOutput(LOG_KEY + "/ReferencePose", referencePose);
        Logger.recordOutput(LOG_KEY + "/ErrorMeters", errorMeters);
        Logger.recordOutput(LOG_KEY + "/ThresholdMeters", replanThresholdMeters);
        Logger.recordOutput(LOG_KEY + "/CooldownSeconds", replanCooldownSeconds);
        Logger.recordOutput(LOG_KEY + "/ReplanCount", replanCount);
    }

    public static class Builder {
        private final Drive drive;
        private PathSource pathSource = PathSource.PATHPLANNER;
        private PathPlannerPath path;
        private String pathName;
        private int choreoSplitIndex = -1;
        private PathConstraints pathfindingConstraints = new PathConstraints(3.0, 3.0, Math.PI * 2.0, Math.PI * 2.0);
        private boolean resetOdom = false;
        private boolean pathfindAtStart = false;
        private double replanThresholdMeters = 0.75;
        private double replanCooldownSeconds = 0.75;
        private double minimumReplanTimeSeconds = 0.25;

        private Builder(Drive drive, PathSource pathSource, String pathName) {
            this.drive = drive;
            this.pathSource = pathSource;
            this.pathName = pathName;
        }

        private Builder(Drive drive, String trajectoryName, int splitIndex) {
            this(drive, PathSource.CHOREO, trajectoryName);
            this.choreoSplitIndex = splitIndex;
        }

        private Builder(Drive drive, PathPlannerPath path) {
            this.drive = drive;
            this.path = path;
        }

        public Builder withPathfindingConstraints(PathConstraints constraints) {
            this.pathfindingConstraints = constraints;
            return this;
        }

        public Builder withResetOdom(boolean resetOdom) {
            this.resetOdom = resetOdom;
            return this;
        }

        public Builder withPathfindAtStart(boolean pathfindAtStart) {
            this.pathfindAtStart = pathfindAtStart;
            return this;
        }

        public Builder withReplanThreshold(double meters) {
            this.replanThresholdMeters = meters;
            return this;
        }

        public Builder withReplanCooldown(double seconds) {
            this.replanCooldownSeconds = seconds;
            return this;
        }

        public Builder withMinimumReplanTime(double seconds) {
            this.minimumReplanTimeSeconds = seconds;
            return this;
        }

        public Command build() {
            try {
                PathPlannerPath loadedPath = path != null ? path : loadPath();
                return new AutoCorrectingPath(
                        drive,
                        loadedPath,
                        pathfindingConstraints,
                        resetOdom,
                        pathfindAtStart,
                        replanThresholdMeters,
                        replanCooldownSeconds,
                        minimumReplanTimeSeconds);
            } catch (Exception e) {
                DriverStation.reportError(
                        "[AutoCorrectingPath] Failed to load "
                                + pathSource.description
                                + " "
                                + pathName
                                + ": "
                                + e.getMessage(),
                        e.getStackTrace());
                return Commands.none();
            }
        }

        private PathPlannerPath loadPath() throws Exception {
            if (pathSource == PathSource.CHOREO) {
                if (choreoSplitIndex >= 0) {
                    return PathPlannerPath.fromChoreoTrajectory(pathName, choreoSplitIndex);
                }
                return PathPlannerPath.fromChoreoTrajectory(pathName);
            }

            return PathPlannerPath.fromPathFile(pathName);
        }
    }

    private enum PathSource {
        PATHPLANNER("PathPlanner path"),
        CHOREO("Choreo trajectory");

        private final String description;

        PathSource(String description) {
            this.description = description;
        }
    }
}
