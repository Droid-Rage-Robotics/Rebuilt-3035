// package frc.robot.commands.autos.pathplanner;

// import static edu.wpi.first.units.Units.*;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.units.measure.*;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.subsystems.drive.SwerveDrive;

// public class LiveAutoCorrectingAuto extends Command {
//     private final SwerveDrive swerve;

//     private final PathConstraints constraints;

//     // Distance from expected pose before replanning
//     private final double replanThresholdMeters;

//     // Prevents replanning spam
//     private final double replanCooldownSeconds;

//     private final boolean doesResetOdo;

//     // Overall path timer
//     private final Timer timer = new Timer();

//     // Tracks time since last replan
//     private final Timer replanCooldownTimer = new Timer();

//     private PathPlannerPath targetPath;
//     private Command activeCommand;

//     public LiveAutoCorrectingAuto(SwerveDrive swerve, AutoConfigs configs) {
//         this.swerve = swerve;
//         this.doesResetOdo = configs.doesResetOdo;
//         this.replanCooldownSeconds = configs.replanCooldown.in(Seconds);
//         this.replanThresholdMeters = configs.replanThreshold.in(Meters);
//         this.constraints = configs.getPathplannerConstraints();

//         // if (doesResetOdo) {
//         //     CommandScheduler.getInstance().schedule(AutoBuilder.resetOdom(targetPath.getStartingHolonomicPose().get()));
//         // }

//         // try {
//         //     if (configs.mirror){
//         //         targetPath = PathPlannerPath.fromPathFile(configs.pathName).mirrorPath();
//         //     } else{
//         //         targetPath = PathPlannerPath.fromPathFile(configs.pathName);
//         //     }

//         //     } catch (Exception e) {
//         //         DriverStation.reportError("Failed to load path: " + configs.pathName, e.getStackTrace());
//         //         targetPath = null;
//         //     }
//     }

//     @Override
//     public void initialize() {
//         if (targetPath == null) {
//             return;
//         }

//         if (doesResetOdo) {
//             CommandScheduler.getInstance().schedule(
//                 AutoBuilder.resetOdom(targetPath.getStartingHolonomicPose().get())
//             );
//         }

//         timer.reset();
//         timer.start();

//         replanCooldownTimer.reset();
//         replanCooldownTimer.start();

//         scheduleNewPathfindCommand();
//     }

//     @Override
//     public void execute() {
//         if (targetPath == null || activeCommand == null) {
//             return;
//         }

//         var trajectoryOptional =
//                 targetPath.getIdealTrajectory(swerve.getRobotConfig());

//         if (trajectoryOptional.isEmpty()) {
//             return;
//         }

//         Pose2d currentPose = swerve.getState().Pose;

//         Pose2d expectedPose =
//                 trajectoryOptional.get()
//                         .sample(timer.get())
//                         .pose;

//         double translationError =
//                 currentPose.getTranslation()
//                         .getDistance(expectedPose.getTranslation());

//         boolean cooldownExpired =
//                 replanCooldownTimer.get() >= replanCooldownSeconds;

//         if (translationError > replanThresholdMeters
//                 && cooldownExpired
//                 && activeCommand.isScheduled()) {

//             DriverStation.reportWarning(
//                     "[AUTO] Replanning. Error: " + translationError,
//                     false);

//             activeCommand.cancel();
//             scheduleNewPathfindCommand();

//             replanCooldownTimer.restart();
//         }
//     }

//     private void scheduleNewPathfindCommand() {
//         activeCommand =
//                 AutoBuilder.pathfindThenFollowPath(
//                         targetPath,
//                         constraints);

//         CommandScheduler.getInstance().schedule(activeCommand);
//     }

//     @Override
//     public boolean isFinished() {
//         return targetPath == null || activeCommand != null && !activeCommand.isScheduled();
//     }

//     @Override
//     public void end(boolean interrupted) {

//         timer.stop();
//         replanCooldownTimer.stop();

//         if (activeCommand != null) {
//             activeCommand.cancel();
//         }
//     }

//     public class AutoConfigs {
//         private final String pathName;
//         private final boolean mirror;
//         private final boolean doesResetOdo;

//         private LinearVelocity maxVelocity;
//         private LinearAcceleration maxAcceleration;

//         private AngularVelocity maxAngularVelocity;
//         private AngularAcceleration maxAngularAcceleration;

//         private Distance replanThreshold = Meters.of(1);
//         private Time replanCooldown = Seconds.of(0.5);

//         private AutoConfigs(String pathName, boolean mirror, boolean doesResetOdo) {
//             this.pathName=pathName;
//             this.mirror=mirror;
//             this.doesResetOdo=doesResetOdo;
//         }

//         public AutoConfigs create(String pathName, boolean mirror, boolean doesResetOdo) {
//             return new AutoConfigs(pathName, mirror, doesResetOdo);
//         }

//         public AutoConfigs withMaxVelocity(LinearVelocity maxVelocity) {
//             this.maxVelocity = maxVelocity;
//             return this;
//         }

//         public AutoConfigs withMaxAcceleration(LinearAcceleration maxAcceleration) {
//             this.maxAcceleration = maxAcceleration;
//             return this;
//         }

//         public AutoConfigs withMaxAngularVelocity(AngularVelocity maxAngularVelocity) {
//             this.maxAngularVelocity = maxAngularVelocity;
//             return this;
//         }

//         public AutoConfigs withMaxAngularAcceleration(AngularAcceleration maxAngularAcceleration) {
//             this.maxAngularAcceleration = maxAngularAcceleration;
//             return this;
//         }

//         public AutoConfigs withReplanCooldown(Time replanCooldown) {
//             this.replanCooldown=replanCooldown;
//             return this;
//         }

//         public AutoConfigs withReplanThreshold(Distance replanThreshold) {
//             this.replanThreshold=replanThreshold;
//             return this;
//         }

//         public PathConstraints getPathplannerConstraints() {
//             return new PathConstraints(
//                     maxVelocity,
//                     maxAcceleration,
//                     maxAngularVelocity,
//                     maxAngularAcceleration);
//         }
//     }
// }