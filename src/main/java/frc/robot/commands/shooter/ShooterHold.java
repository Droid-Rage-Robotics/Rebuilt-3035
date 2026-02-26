package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.HubShooterMath.*;

import java.util.function.Supplier;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants.FieldConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.HubShooterMath.ShotData;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterHold extends Command {
    private final Shooter shooter;    

    private final Translation3d hubPose;

    private final Supplier<SwerveDriveState> robot;

    private Translation3d predictedTarget;
    private ShotData shot;
    private Time timeOfFlight;
    
    public ShooterHold(SwerveDrive drive, Shooter shooter) {
        this.shooter = shooter;
        this.robot = drive::getState;

        var alliance = DriverStation.getAlliance();
        this.hubPose = alliance.isPresent() && alliance.get() == Alliance.Red 
            ? FieldConstants.HUB_RED 
            : FieldConstants.HUB_BLUE;


        addRequirements(
            shooter.getHood(),
            shooter.getShooterWheel(),
            shooter.getTurret());
    }

    @Override
    public void initialize() {
        var state = robot.get();
        // Perform initial estimation (assuming unmoving robot) to get time of flight estimate
        shot = calculateShotFromFunnelClearance(state.Pose, hubPose, hubPose);
        Distance distance = getDistanceToTarget(state.Pose, hubPose);
        timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), distance);
        predictedTarget = hubPose;

        shooter.getTurret().setGoalAngle(Rotation2d.kZero);
        shooter.getShooterWheel().setTargetVelocity(Shooter.IDLE_VELOCITY);
    }

    @Override
    public void execute() {
        var state = robot.get();
        shooter.getHood().setGoalAngle(new Rotation2d(shot.getHoodAngle()));
        
        // Iterate the process, getting better time of flight estimations and updating the predicted target accordingly
        predictedTarget = predictTargetPos(hubPose, state.Speeds, timeOfFlight);
        shot = calculateShotFromFunnelClearance(state.Pose, hubPose, predictedTarget);
        timeOfFlight = calculateTimeOfFlight(
            shot.getExitVelocity(), shot.getHoodAngle(), getDistanceToTarget(state.Pose, predictedTarget));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.getHood().setGoalAngle(Rotation2d.kZero);
        shooter.getTurret().setGoalAngle(Rotation2d.kZero);
        shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.zero());
    }
}
