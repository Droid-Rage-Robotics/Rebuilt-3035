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
import frc.utility.DRAreaManager;


public class DRShooter extends Command {
    private final Shooter shooter;
    private final SwerveDrive drive;
    
    private Translation2d hubPose;
    private Translation2d goalPose;
    private double distanceRobotToGoal;

    public DRShooter(SwerveDrive drive, Shooter shooter) {
        this.shooter = shooter;
        this.drive = drive;

        //this part does NOT initialize Positions correctly; Check initialize()
        // if(DroidRageConstants.alliance == Alliance.Red){
        //     this.hubPose = FieldConstants.HUB_RED;
        //     this.alliancePose = FieldConstants.ALLIANCE_RED;
        // } else if (DroidRageConstants.alliance == Alliance.Blue){
        //     this.hubPose = FieldConstants.HUB_BLUE;
        //     this.alliancePose = FieldConstants.ALLIANCE_BLUE;
        // }
        
        this.goalPose = this.hubPose;

        addRequirements(
            shooter.getHood(),
            shooter.getShooterWheel(),
            shooter.getTurret());
    }

    @Override
    public void initialize(){
        if(DroidRageConstants.alliance == Alliance.Red){
            this.hubPose = FieldConstants.HUB_RED;
            // this.alliancePose = FieldConstants.ALLIANCE_RED;
        } else if (DroidRageConstants.alliance == Alliance.Blue){
            this.hubPose = FieldConstants.HUB_BLUE;
            // this.alliancePose = FieldConstants.ALLIANCE_BLUE;
        }
    }
    
    @Override 
    public void execute(){
        
        // this.hubPose = FieldConstants.HUB_RED;

        switch(DRAreaManager.getCurrentZone()){
            case ALLIANCE_ZONE:
                this.goalPose = this.hubPose;
                break;
            case BETWEEN:
                this.goalPose = this.hubPose;
                break;
            case NEUTRAL, OPPOSITION:
                // this.goalPose = this.hubPose;
                // this.goalPose = new Translation2d(this.alliancePose.getX(), drive.getState().Pose.getY());
                break;
        }

        // Get the predicted robot pose based on current velocity to improve targeting while moving for 1 Second ahead
        Pose2d lookAheadPose = Shooter.predictPosePos(
            drive.getState().Pose, 
            drive.getCurrentRobotChassisSpeeds(),
            Shooter.hoodMap.get(Shooter.getDistanceToHub(drive.getState().Pose, goalPose)));
        distanceRobotToGoal = Shooter.getDistanceToHub(lookAheadPose, goalPose);//TODO: Output Distance

        // distanceRobotToGoal = getDistanceToHub(drive.getState().Pose, goalPose);//TODO: Output Distance
        // System.out.println(distanceRobotToGoal);

        if (!DroidRageConstants.isShooterManual) {
            switch(DRAreaManager.getCurrentZone()){
                case ALLIANCE_ZONE:
                    shooter.getTurret().setGoalAngle(Shooter.calculateAzimuthAngle(drive.getState().Pose, hubPose));
                    // System.out.println(calculateAzimuthAngle(drive.getState().Pose, hubPose).in(Degrees));

                    shooter.getHood().setGoalAngle(Degrees.of(Shooter.hoodMap.get(distanceRobotToGoal)));
                    shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(Shooter.flywheelSpeedMap.get(distanceRobotToGoal)));
                    break;
                case NEUTRAL,OPPOSITION:
                    shooter.getHood().setGoalAngle(Degrees.zero());
                    shooter.getShooterWheel().setTargetVelocity(Shooter.IDLE_VELOCITY);
                    // shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(0));


                    break;
                case BETWEEN:
                    shooter.getTurret().setGoalAngle(Shooter.calculateAzimuthAngle(drive.getState().Pose, hubPose));
                    shooter.getHood().setGoalAngle(Degrees.zero());
                    shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(Shooter.flywheelSpeedMap.get(distanceRobotToGoal)));
                    break;
            }
        }
    }
}