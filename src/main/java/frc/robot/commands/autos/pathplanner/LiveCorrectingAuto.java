package frc.robot.commands.autos.pathplanner;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autos.pathplanner.LiveAutoCorrectingAuto.AutoConfigs;
import frc.robot.subsystems.drive.SwerveDrive;

public class LiveCorrectingAuto {
    private SwerveDrive swerve;
    private PathConstraints constraints;

    private Distance replanThreshold = Meters.of(1);
    private Time replanCooldown = Seconds.of(0.5);

    private boolean doesResetOdo, mirror;

    // Overall path timer
    private Timer timer = new Timer();

    // Tracks time since last replan
    private Timer replanCooldownTimer = new Timer();

    private PathPlannerPath targetPath;
    private Command activeCommand;
    private String pathName;
    private LinearVelocity maxVelocity;
    private LinearAcceleration maxAcceleration;
    private AngularVelocity maxAngularAcceleration;

    public LiveCorrectingAuto(){

    }

    public static VelocityBuilder create(String pathName){
        LiveCorrectingAuto pf = new LiveCorrectingAuto();
        pf.pathName = pathName;
        return pf.new VelocityBuilder();
    }

    public class VelocityBuilder{
        public AccelerationBuilder withMaxVelocity(LinearVelocity velocity) {
            LiveCorrectingAuto.this.maxVelocity = velocity;
            return new AccelerationBuilder();
        }
    }

    public class AccelerationBuilder{
        public ResetOdoBuilder withMaxAcceleration(LinearAcceleration acceleration) {
            LiveCorrectingAuto.this.maxAcceleration = acceleration;
            return new ResetOdoBuilder();
        }
    }
    public class AngularAccelerationBuilder{
        public ResetOdoBuilder withMaxAngularAcceleration(AngularVelocity angularAcceleration) {
            LiveCorrectingAuto.this.maxAngularAcceleration = angularAcceleration;
            return new ResetOdoBuilder();
        }
    }

    public class ResetOdoBuilder {
        public MirrorBuilder withResetOdo(boolean doesResetOdo) {
            LiveCorrectingAuto.this.doesResetOdo = doesResetOdo;
            return new MirrorBuilder();
        }
    }
    public class MirrorBuilder {
        public PathBuilder withMirror(boolean mirror) {
            LiveCorrectingAuto.this.mirror = mirror;
            return new PathBuilder();
        }
    }

    public class PathBuilder{
        public Command build(){
            timer.reset();
            timer.start();

            replanCooldownTimer.reset();
            replanCooldownTimer.start();

            scheduleNewPathfindCommand();

            try {
                PathPlannerPath path;
                if(LiveCorrectingAuto.this.mirror){
                    path = PathPlannerPath.fromPathFile(LiveCorrectingAuto.this.pathName).mirrorPath();
                }else{
                    path = PathPlannerPath.fromPathFile(LiveCorrectingAuto.this.pathName);
                }

                if (doesResetOdo) {
                    return new SequentialCommandGroup(
                        AutoBuilder.resetOdom(path.getStartingHolonomicPose().get()),
                        AutoBuilder.followPath(path)
                    );
                } else {
                    return AutoBuilder.followPath(path).finallyDo(
                        (interrupted) -> {
                            timer.stop();
                            replanCooldownTimer.stop();

                            if (activeCommand != null) {
                                activeCommand.cancel();
                            }
                        }
                    ).beforeStarting(
                        () -> {
                            targetPath = path;
                            constraints = new PathConstraints(
                                LiveCorrectingAuto.this.maxVelocity.get(),
                                LiveCorrectingAuto.this.maxAcceleration.get(),
                                LiveCorrectingAuto.this.maxAngularAcceleration.get(),
                                Double.POSITIVE_INFINITY
                            );
                        }
                    );
                }
            } catch (Exception e) {
                DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                return Commands.none();
            }
        }
        
        public PathBuilder withReplanCooldown(Time replanCooldown){
            LiveCorrectingAuto.this.replanCooldown=replanCooldown;
            return this;
        }

        public PathBuilder withReplanThreshold(Distance replanThreshold) {
            LiveCorrectingAuto.this.replanThreshold=replanThreshold;
            return this;
        }

        private void scheduleNewPathfindCommand() {
            activeCommand =
                    AutoBuilder.pathfindThenFollowPath(
                            targetPath,
                            constraints);

            CommandScheduler.getInstance().schedule(activeCommand);
        }
    }

}
