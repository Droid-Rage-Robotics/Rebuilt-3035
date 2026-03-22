package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.DroidRageConstants.FieldConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.HubShooterMath;
import frc.robot.subsystems.shooter.Shooter;

public class DRShooter extends Command{

    private final Shooter shooter;    
    private final Translation3d hubPose;
    private final SwerveDrive drive;
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
        hoodMap.put(2.8, 10.0); //Short Hub
        hoodMap.put(3.47, 5.57); //Trench
    }

    public DRShooter(SwerveDrive drive, Shooter shooter) {
        this.shooter = shooter;
        // this.robot = drive::getState;
        this.drive = drive;

        this.hubPose = DroidRageConstants.alliance == Alliance.Red 
            ? FieldConstants.HUB_RED 
            : FieldConstants.HUB_BLUE;


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
        //Get the predicted robot pose based on current velocity to improve targeting while moving for 1 Second ahead
        // Pose2d lookAheadPose = predictPosePos(
        //     drive.getState().Pose, 
        //     drive.getCurrentRobotChassisSpeeds());

        double distanceRobotToHub = getDistanceToHub(drive.getState().Pose, hubPose);

        shooter.getTurret().setGoalAngle(HubShooterMath.calculateTurretAngle(drive.getState().Pose, hubPose));
        shooter.getHood().setGoalAngle(Rotation2d.fromDegrees(hoodMap.get(distanceRobotToHub)));
        shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(flywheelSpeedMap.get(distanceRobotToHub)));

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

}
