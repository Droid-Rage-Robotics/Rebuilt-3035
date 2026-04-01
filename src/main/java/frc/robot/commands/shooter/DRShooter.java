package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.DroidRageConstants.FieldConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret;
import frc.utility.DRAreaManager;

public class DRShooter extends Command{

    private final Shooter shooter;    
    private final Translation3d hubPose, alliancePose;
    private Translation3d goalPose;
    private final SwerveDrive drive;
    private double distanceRobotToGoal;
    public static final Transform3d ROBOT_TO_TURRET_TRANSFORM =
        new Transform3d(new Translation3d(Inches.zero(), Inches.of(-13.25), Inches.of(6.613)), 
            new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));//-32.5
    private static final InterpolatingDoubleTreeMap hoodMap =
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
        new InterpolatingDoubleTreeMap();
    static{
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
        switch(DRAreaManager.getCurrentZone()){
            case ALLIANCE_ZONE:
                this.goalPose = this.hubPose;
                break;
            case BETWEEN:
                this.goalPose = this.hubPose;
                break;
            case NEUTRAL, OPPOSITION:
                this.goalPose = this.hubPose;
                // this.goalPose = this.alliancePose;
                break;
        }

        // Get the predicted robot pose based on current velocity to improve targeting while moving for 1 Second ahead
        // Pose2d lookAheadPose = predictPosePos(
        //     drive.getState().Pose, 
        //     drive.getCurrentRobotChassisSpeeds());
        // distanceRobotToGoal = getDistanceToHub(lookAheadPose, goalPose);//TODO: Output Distance

        distanceRobotToGoal = getDistanceToHub(drive.getState().Pose, goalPose);//TODO: Output Distance

        if (!DroidRageConstants.isShooterManual) {
            switch(DRAreaManager.getCurrentZone()){
                case ALLIANCE_ZONE,NEUTRAL,OPPOSITION:
                    shooter.getTurret().setGoalAngle(calculateAzimuthAngle(drive.getState().Pose, hubPose));
                    // shooter.getTurret().setGoalAngle(calculateTurretAngle(drive.getState().Pose, hubPose));
                    // shooter.getTurret().setGoalAngle(getTurretSetpoint());

                    shooter.getHood().setGoalAngle(Rotation2d.fromDegrees(hoodMap.get(distanceRobotToGoal)));
                    shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(flywheelSpeedMap.get(distanceRobotToGoal)));
                    break;
                case BETWEEN:
                    shooter.getTurret().setGoalAngle(calculateAzimuthAngle(drive.getState().Pose, hubPose));
                    shooter.getHood().setGoalAngle(Rotation2d.kZero);
                    shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(flywheelSpeedMap.get(distanceRobotToGoal)));
                    break;
            }
        }
    }
    
    public static double getDistanceToHub(Pose2d robotPose, Translation3d target){
        Translation2d target2d = new Translation2d(target.getX(), target.getY());
        return robotPose.getTranslation().getDistance(target2d);
    }

    public static Pose2d predictPosePos(Pose2d currentPose, ChassisSpeeds fieldSpeeds) {
        double predictedX = currentPose.getX() - fieldSpeeds.vxMetersPerSecond;
        double predictedY = currentPose.getY() - fieldSpeeds.vyMetersPerSecond;

        // double turretVelocityX =
        //         fieldVelocity.vxMetersPerSecond
        //                 + fieldVelocity.omegaRadiansPerSecond
        //                         * (robotToTurret.getY() * Math.cos(robotAngle)
        //                                 - robotToTurret.getX() * Math.sin(robotAngle));
        // double turretVelocityY =
        //         fieldVelocity.vyMetersPerSecond
        //                 + fieldVelocity.omegaRadiansPerSecond
        //                         * (robotToTurret.getX() * Math.cos(robotAngle)
        //                                 - robotToTurret.getY() * Math.sin(robotAngle));

        return new Pose2d(predictedX, predictedY, currentPose.getRotation());
    }

    /**
     * calculates the angle of a turret relative to the robot to hit a target
     * @param robot robot pos
     * @param target target pos
     * @return new turret angle measure
     */
    public static Angle calculateAzimuthAngle(Pose2d robot, Translation3d target) {
        Translation2d turretTranslation = new Pose3d(robot)
                .transformBy(ROBOT_TO_TURRET_TRANSFORM)
                .toPose2d()
                .getTranslation();

        Translation2d direction = target.toTranslation2d().minus(turretTranslation);

        double rawAngle = direction.getAngle()
            // .minus(robot.getRotation()) //Why are we  using the robot angle again after its initial use? 
            // ^^ Is problem?
            .plus(Rotation2d.fromDegrees(28)) //The Offset for Starting Turret at Angle
            .getRadians();

        // Wrap to [0, 2π] first, then you can clamp in setGoalAngle
        return Radians.of(MathUtil.inputModulus(rawAngle, 0, 2 * Math.PI));
    }

    //https://github.com/FRC-8557-Conquera/Frc2026autoaimtry/blob/main/src/main/java/frc/robot/subsystems/shooter/ShooterSubsystem.java#L133
    public Angle getTurretSetpoint() {
        Pose2d robotPose = drive.getState().Pose;
        Translation2d turretOffset = new Translation2d(Inches.of(0), Inches.of(-13.25));  //Inches.zero(), Inches.of(-13.25), Inches.of(6.613)
        Translation2d turretFieldPosition = robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));

        Translation2d targetTranslation =  hubPose.toTranslation2d();
        Rotation2d fieldAngle = targetTranslation.minus(turretFieldPosition).getAngle();
        double setpointDeg = fieldAngle.minus(robotPose.getRotation()).getDegrees();
        
        return Degrees.of(MathUtil.inputModulus(setpointDeg, 0, 360));
       
    }

    // public Angle getSpectrumTurretAngle(Pose2d estimatedPose) {
    //     Pose2d turretPose = estimatedPose.transformBy(new Transform2d(Inches.of(0), Inches.of(-13.25), new Rotation2d()));
    //     // double turretToTargetDistance = target.getDistance(turretPose.getTranslation());

    //     return Degrees.of(0);
    // }

    public static Angle calculateTurretAngle(Pose2d robot, Translation3d target) {
        // 1. Convert robot pose (2D) to 3D
        Pose3d robotPose3d = new Pose3d(robot);

        // 2. Get turret pose in field coordinates
        Pose3d turretPose3d = robotPose3d.transformBy(ROBOT_TO_TURRET_TRANSFORM);

        // 3. Extract turret position in 2D (ignore height for azimuth)
        Translation2d turretPos = turretPose3d.toPose2d().getTranslation();

        // 4. Target position in 2D
        Translation2d targetPos = new Translation2d(target.getX(), target.getY());

        // 5. Vector from turret to target
        Translation2d delta = targetPos.minus(turretPos);

        // 6. Angle from turret to target (field-relative)
        Rotation2d fieldAngle = delta.getAngle();

        // 7. Convert to robot-relative turret angle
        Rotation2d turretAngle = fieldAngle.minus(robot.getRotation());

        return Radians.of(MathUtil.inputModulus(turretAngle.getRadians(), 0, 2 * Math.PI));
    }

}