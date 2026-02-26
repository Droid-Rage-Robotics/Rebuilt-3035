package frc.robot.commands;

import static frc.robot.subsystems.shooter.HubShooterMath.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.HubShooterMath.ShotData;

public class ShooterScore extends Command{
    private volatile Translation3d predictedTarget;
    private volatile ShotData shot;
    private volatile Time timeOfFlight;

    private volatile SwerveDriveState robot;


    public ShooterScore(SwerveDrive drive) {
        robot = drive.getState();
    }

    @Override
    public void initialize() {
        // Perform initial estimation (assuming unmoving robot) to get time of flight estimate
        shot = calculateShotFromFunnelClearance(robot.get().Pose, target, target);
        Distance distance = getDistanceToTarget(robot.get().Pose, target);
        timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), distance);
        Translation3d predictedTarget = target;
    }
    @Override
    public void execute() {
        robot = drive.getState;
        // Iterate the process, getting better time of flight estimations and updating the predicted target accordingly
        predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
        shot = calculateShotFromFunnelClearance(robot, target, predictedTarget);
        timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), getDistanceToTarget(robot, predictedTarget));
        
    }
}
