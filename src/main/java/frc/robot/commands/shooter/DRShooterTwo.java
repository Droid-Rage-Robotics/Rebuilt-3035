package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.DroidRageConstants;
import frc.robot.DroidRageConstants.FieldConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.HubShooterMath;
import frc.robot.subsystems.shooter.Shooter;
import frc.utility.DRAreaManager;

public class DRShooterTwo extends Command{

    private final Shooter shooter;    
    private final Translation3d hubPose, alliancePose;
    private Translation3d goalPose;
    private final SwerveDrive drive;
    private double distanceRobotToGoal = 0.0;
    // private final Supplier<SwerveDriveState> robot;

    // private static final InterpolatingTreeMap<Distance, Rotation2d> hoodMap =
    //     new InterpolatingTreeMap<Distance, Rotation2d>(null, Rotation2d::interpolate); //TODO:Fix
     private static final InterpolatingDoubleTreeMap hoodMap =
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
        new InterpolatingDoubleTreeMap();
    static {
        //Distance to hub (in) -> flywheel speed (RPM)
        // flywheelSpeedMap.put(1.1, 1.1);
        // flywheelSpeedMap.put(1.1, 1.1);
        // flywheelSpeedMap.put(1.1, 1.1);
        // flywheelSpeedMap.put(1.1, 1.1);
        // flywheelSpeedMap.put(1.1, 1.1);

        //meters to hub -> flywheel speed (RPM)
        flywheelSpeedMap.put(0.96, 20.0);
        flywheelSpeedMap.put(1.16, 30.0);
        flywheelSpeedMap.put(1.58, 40.0);
        flywheelSpeedMap.put(2.07, 48.0);//Short Hub
        flywheelSpeedMap.put(2.37, 50.0);
        flywheelSpeedMap.put(2.47, 52.0);
        flywheelSpeedMap.put(2.70, 53.0);
        flywheelSpeedMap.put(2.94, 55.0);
        flywheelSpeedMap.put(3.47, 57.5); //Trench
        flywheelSpeedMap.put(3.92, 60.0);
        flywheelSpeedMap.put(4.35, 67.0);
        flywheelSpeedMap.put(4.84, 70.0);

        //Distance to hub (m) -> hood angle (degrees)
        hoodMap.put(1.0, 15.0); //Max Pos next to Hub
        hoodMap.put(2.8, 10.0); //Short Hub
        hoodMap.put(3.47, 5.57); //Trench
        hoodMap.put(4.5, 0.0); //Against Field wall Straighy


    }

    public DRShooterTwo(SwerveDrive drive, Shooter shooter) {
        this.shooter = shooter;
        // this.robot = drive::getState;
        this.drive = drive;

        this.hubPose = DroidRageConstants.alliance == Alliance.Red 
            ? FieldConstants.HUB_RED 
            : FieldConstants.HUB_BLUE;
        this.alliancePose = DroidRageConstants.alliance == Alliance.Red 
            ? FieldConstants.ALLIANCE_RED 
            : FieldConstants.ALLIANCE_BLUE;
        this.goalPose = this.hubPose;


        addRequirements(
            shooter.getHood(),
            shooter.getShooterWheel(),
            shooter.getTurret());

        
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute(){
        // Get the predicted robot pose based on current velocity to improve targeting while moving for 1 Second ahead
        // Pose2d lookAheadPose = predictPosePos(
        //     drive.getState().Pose, 
        //     drive.getCurrentRobotChassisSpeeds());

        switch(DRAreaManager.getCurrentZone()){
            case ALLIANCE_ZONE:
                this.goalPose = this.hubPose;
                break;
            case BETWEEN:
                break;
            case NEUTRAL:
                this.goalPose = this.alliancePose;
                break;
            case OPPOSITION:
                this.goalPose = this.alliancePose;
                break;
        }
        distanceRobotToGoal = getDistanceToHub(drive.getState().Pose, goalPose);

        // shooter.getTurret().setGoalAngle(getTurretAngleDegrees(drive.getState().Pose, hubPose));
        DRAreaManager.inAllianceZone().or(DRAreaManager.inNeutral()).whileTrue(new ParallelCommandGroup(
            shooter.getTurret().setTargetPositionCommand(HubShooterMath.calculateAzimuthAngle(drive.getState().Pose, hubPose)),
            shooter.getHood().setTargetPositionCommand(Rotation2d.fromDegrees(hoodMap.get(distanceRobotToGoal))),
            shooter.getShooterWheel().setTargetVelocityCommand(RotationsPerSecond.of(flywheelSpeedMap.get(distanceRobotToGoal)))
        ));

        DRAreaManager.inOpposition();

        DRAreaManager.inBetween().onTrue(
            new ParallelCommandGroup(
                shooter.getHood().setTargetPositionCommand(Rotation2d.kZero) 
            )
        );

    }
    
    public double getDistanceToHub(Pose2d robotPose, Translation3d target){
        Translation2d target2d = new Translation2d(target.getX(), target.getY());
        return robotPose.getTranslation().getDistance(target2d);
    }

    public static Pose2d predictPosePos(Pose2d currentPose, ChassisSpeeds fieldSpeeds) {
        double predictedX = currentPose.getX() - fieldSpeeds.vxMetersPerSecond;
        double predictedY = currentPose.getY() - fieldSpeeds.vyMetersPerSecond;

        return new Pose2d(predictedX, predictedY, currentPose.getRotation());
    }
    public Angle getTurretAngleDegrees( Pose2d robotPose, Translation3d target) {
        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();

        // Angle from robot to goal (field-relative)
        Rotation2d goalAngle = Rotation2d.fromRadians(Math.atan2(dy, dx));

        // Turret relative Radian angle
        double turretAngle = goalAngle.getRadians() - robotPose.getRotation().getRadians()+Math.PI;

        // Normalize to 0 → 2π
        turretAngle = Math.atan2(Math.sin(turretAngle), Math.cos(turretAngle));

        System.out.println(turretAngle);
        return Radians.of(turretAngle);
    }

}
