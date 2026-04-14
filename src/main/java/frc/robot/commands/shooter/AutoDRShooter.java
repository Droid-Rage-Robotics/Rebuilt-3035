package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.DroidRageConstants.FieldConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.Shooter;

public class AutoDRShooter extends Command {
    private final Shooter shooter;
    private final SwerveDrive drive;

    private Translation2d goalPose;
    private double distanceRobotToGoal;

    public AutoDRShooter(SwerveDrive drive, Shooter shooter) {
        this.shooter = shooter;
        this.drive = drive;

        addRequirements(
            shooter.getHood(),
            shooter.getShooterWheel(),
            shooter.getTurret());
    }

    @Override
    public void initialize(){
        if(DroidRageConstants.alliance == Alliance.Red){
            this.goalPose = FieldConstants.HUB_RED;
            // this.alliancePose = FieldConstants.ALLIANCE_RED;
        } else if (DroidRageConstants.alliance == Alliance.Blue){
            this.goalPose = FieldConstants.HUB_BLUE;
            // this.alliancePose = FieldConstants.ALLIANCE_BLUE;
        }
    }
    
    @Override 
    public void execute(){
        // Get the predicted robot pose based on current velocity to improve targeting while moving for 1 Second ahead
        Pose2d lookAheadPose = Shooter.predictPosePos(
            drive.getState().Pose, 
            goalPose,
            drive.getCurrentRobotChassisSpeeds(),
            Shooter.timeOfFlightMap.get(Shooter.getDistanceToHub(drive.getState().Pose, goalPose)));

            // Shooter.timeOffFlightMap.get(Shooter.getDistanceToHub(drive.getState().Pose, goalPose)));
        
        distanceRobotToGoal = Shooter.getDistanceToHub(lookAheadPose, goalPose);

        shooter.getTurret().setGoalAngle(Shooter.calculateAzimuthAngle(drive.getState().Pose, goalPose));
        shooter.getHood().setGoalAngle(Degrees.of(Shooter.hoodMap.get(distanceRobotToGoal)));
        shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(Shooter.flywheelSpeedMap.get(distanceRobotToGoal)));
    }
}
