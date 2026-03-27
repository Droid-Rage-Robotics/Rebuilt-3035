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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        // flywheelSpeedMap.put(0.96, 20.0);
        // flywheelSpeedMap.put(1.16, 30.0);
        // flywheelSpeedMap.put(1.58, 40.0);
        // flywheelSpeedMap.put(2.07, 44.0);//Short Hub
        // flywheelSpeedMap.put(2.37, 48.0);
        // flywheelSpeedMap.put(2.47, 50.0);
        // flywheelSpeedMap.put(2.70, 53.0);
        // flywheelSpeedMap.put(2.94, 55.0);
        // flywheelSpeedMap.put(3.47, 57.5); //Trench
        // flywheelSpeedMap.put(3.92, 60.0);
        // flywheelSpeedMap.put(4.35, 67.0);
        // flywheelSpeedMap.put(4.84, 70.0);

        // //Distance to hub (m) -> hood angle (degrees)
        // hoodMap.put(1.0, 0.0); //Max Pos next to Hub
        // hoodMap.put(2.8, 9.0); //Short Hub
        // hoodMap.put(3.47, 15.0); //Trench SA: 5.57
        // hoodMap.put(4.5, 19.0); //Against Field wall Straighy

        //Don't Delete
        /*  hub
            pos 1.25
            hood 0 - 2
            vel 40

            bump
            pos 1.89
            setpoint - is
            hood 11.2 - 10.6
            vel 42

            pos 3.05
            hood 12,1 - 12.3
            vel 47

            pos 4.27
            setpoint -is
            hood 12.75-12.2
            vel - 50.2 
        */

        flywheelSpeedMap.put(1.25,40.0);
        flywheelSpeedMap.put(1.89,42.0);
        flywheelSpeedMap.put(3.05,47.0);
        flywheelSpeedMap.put(4.27,50.2);

        hoodMap.put(1.25,.0);
        hoodMap.put(1.89,11.2);
        hoodMap.put(3.05,12.1);
        hoodMap.put(4.27,12.75);

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
//Test
    @Override 
    public void execute(){
        // Get the predicted robot pose based on current velocity to improve targeting while moving for 1 Second ahead
        // Pose2d lookAheadPose = predictPosePos(
        //     drive.getState().Pose, 
        //     drive.getCurrentRobotChassisSpeeds());

        double distanceRobotToHub = getDistanceToHub(drive.getState().Pose, hubPose);
        // System.out.println("distance:"+distanceRobotToHub);
        //TODO: Output Distance
                // SmartDashboard.putData("Shooter/Distance", distanceRobotToHub);

        shooter.getTurret().setGoalAngle(HubShooterMath.calculateAzimuthAngle(drive.getState().Pose, hubPose));
        // shooter.getTurret().setGoalAngle(getTurretAngleDegrees(drive.getState().Pose, hubPose));

        System.out.println(HubShooterMath.calculateAzimuthAngle(drive.getState().Pose, hubPose));

        shooter.getHood().setGoalAngle(Rotation2d.fromDegrees(hoodMap.get(distanceRobotToHub)));
        shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(flywheelSpeedMap.get(distanceRobotToHub)));
        // shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(20));
    }
    
    public static double getDistanceToHub(Pose2d robotPose, Translation3d target){
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
