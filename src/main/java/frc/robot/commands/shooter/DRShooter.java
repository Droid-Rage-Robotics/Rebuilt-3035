package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.DroidRageConstants.FieldConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.Shooter;
import frc.utility.DRAreaManager;


public class DRShooter extends Command{

    private final Shooter shooter;    
    private  Translation2d hubPose, alliancePose;
    private Translation2d goalPose;
    private final SwerveDrive drive;
    private double distanceRobotToGoal;
    public static final Transform2d ROBOT_TO_TURRET_TRANSFORM =
        new Transform2d(
            new Translation2d(Inches.zero(), Inches.of(-9.32)), //-13.25
            new Rotation2d(Degrees.of(0)));//-32.5
    private static final InterpolatingDoubleTreeMap hoodMap =
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOffFlightMap =
        new InterpolatingDoubleTreeMap();
    static{
        // Springs Old
        // flywheelSpeedMap.put(1.0,42.5);
        // flywheelSpeedMap.put(2.15,47.0);
        // flywheelSpeedMap.put(3.28,50.6);
        // flywheelSpeedMap.put(4.10,54.5);
        // flywheelSpeedMap.put(5.30,58.0);


        // hoodMap.put(1.0,.0);
        // hoodMap.put(2.15, 5.77);
        // hoodMap.put(3.28,7.6);
        // hoodMap.put(4.10,9.8);
        // hoodMap.put(5.30,11.5);


        flywheelSpeedMap.put(1.5,41.5);
        flywheelSpeedMap.put(2.0,42.7);
        flywheelSpeedMap.put(2.9,46.0);
        flywheelSpeedMap.put(4.20,50.0);
        flywheelSpeedMap.put(5.10,53.5);
        flywheelSpeedMap.put(5.6,58.1);



        hoodMap.put(1.5,2.85);
        hoodMap.put(2.00, 7.8);
        hoodMap.put(2.9,10.7);
        hoodMap.put(4.2,16.8);
        hoodMap.put(5.10,18.5);
        hoodMap.put(5.6,19.5);


        timeOffFlightMap.put(5.68,1.16);
        timeOffFlightMap.put(4.55,1.12);
        timeOffFlightMap.put(3.15,1.11);
        timeOffFlightMap.put(1.88,1.01);
        timeOffFlightMap.put(1.38,0.9);


        //Home Positions
        // flywheelSpeedMap.put(1.25,40.0);
        // flywheelSpeedMap.put(1.89,42.0);
        // flywheelSpeedMap.put(3.05,47.0);
        // flywheelSpeedMap.put(4.27,50.2);

        // hoodMap.put(1.25,.0);
        // hoodMap.put(1.89,11.2);
        // hoodMap.put(3.05,12.1);
        // hoodMap.put(4.27,12.75);
    }

    public DRShooter(SwerveDrive drive, Shooter shooter) {
        this.shooter = shooter;
        this.drive = drive;

        //this part does NOT initialize Positions correctly; Check initialize()
        if(DroidRageConstants.alliance == Alliance.Red){
            this.hubPose = FieldConstants.HUB_RED;
            this.alliancePose = FieldConstants.ALLIANCE_RED;
        } else if (DroidRageConstants.alliance == Alliance.Blue){
            this.hubPose = FieldConstants.HUB_BLUE;
            this.alliancePose = FieldConstants.ALLIANCE_BLUE;
        }
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
            this.alliancePose = FieldConstants.ALLIANCE_RED;
        } else if (DroidRageConstants.alliance == Alliance.Blue){
            this.hubPose = FieldConstants.HUB_BLUE;
            this.alliancePose = FieldConstants.ALLIANCE_BLUE;
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
        Pose2d lookAheadPose = predictPosePos(
            drive.getState().Pose, 
            drive.getCurrentRobotChassisSpeeds(),
            timeOffFlightMap.get(getDistanceToHub(drive.getState().Pose, goalPose)));
        distanceRobotToGoal = getDistanceToHub(lookAheadPose, goalPose);//TODO: Output Distance

        // distanceRobotToGoal = getDistanceToHub(drive.getState().Pose, goalPose);//TODO: Output Distance
        System.out.println(distanceRobotToGoal);

        if (!DroidRageConstants.isShooterManual) {
            switch(DRAreaManager.getCurrentZone()){
                case ALLIANCE_ZONE:
                    shooter.getTurret().setGoalAngle(calculateAzimuthAngle(drive.getState().Pose, hubPose));
                    // System.out.println(calculateAzimuthAngle(drive.getState().Pose, hubPose).in(Degrees));

                    shooter.getHood().setGoalAngle(Rotation2d.fromDegrees(hoodMap.get(distanceRobotToGoal)));
                    shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(flywheelSpeedMap.get(distanceRobotToGoal)));
                    break;
                case NEUTRAL,OPPOSITION:
                    shooter.getHood().setGoalAngle(Rotation2d.kZero);
                    shooter.getShooterWheel().setTargetVelocity(Shooter.IDLE_VELOCITY);
                    // shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(0));


                    break;
                case BETWEEN:
                    shooter.getTurret().setGoalAngle(calculateAzimuthAngle(drive.getState().Pose, hubPose));
                    shooter.getHood().setGoalAngle(Rotation2d.kZero);
                    shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(flywheelSpeedMap.get(distanceRobotToGoal)));
                    break;
            }
        }
    }
    
    public static double getDistanceToHub(Pose2d robotPose, Translation2d target){
        // Translation2d target2d = new Translation2d(target.getX(), target.getY());
        return target.getDistance(robotPose.getTranslation());
        // return robotPose.getTranslation().getDistance(target2d);
    }

    public static Pose2d predictPosePos(Pose2d currentPose, ChassisSpeeds fieldSpeeds, double timeOffFlight) {
        double predictedX = currentPose.getX() + fieldSpeeds.vxMetersPerSecond * timeOffFlight;
        double predictedY = currentPose.getY() + fieldSpeeds.vyMetersPerSecond * timeOffFlight;

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
    public static Angle calculateAzimuthAngle(Pose2d robot, Translation2d target) {
        Translation2d turretTranslation = robot
                .transformBy(ROBOT_TO_TURRET_TRANSFORM)
                .getTranslation();

        Translation2d direction = target.minus(turretTranslation);

        double rawAngle = direction.getAngle()
            .minus(robot.getRotation())
            // .plus(Rotation2d.fromDegrees(28)) //The Offset for Starting Turret at Angle
            .getRadians();

        // Wrap to [0, 2π] first, then you can clamp in setGoalAngle
        return Radians.of(MathUtil.inputModulus(rawAngle, 0, 2 * Math.PI));
    }

    //https://blog.eeshwark.com/robotblog/shooting-on-the-fly
//     public void update(Pose2d robotPose, ChassisSpeeds robotSpeed) {

//         // 1. LATENCY COMP
//         double latency = 0.15; // Tuned constant
//         Translation2d futurePos = robotPose.getTranslation().plus(
//             new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
//         );

//         // 2. GET TARGET VECTOR
//         Translation2d goalLocation = FieldConstants.HUB_RED;
//         Translation2d targetVec = goalLocation.minus(futurePos);
//         double dist = targetVec.getNorm();

//         // 3. CALCULATE IDEAL SHOT (Stationary)
//         // Note: This returns HORIZONTAL velocity component
//         double idealHorizontalSpeed = flywheelSpeedMap.get(dist);

//         // 4. VECTOR SUBTRACTION
//         Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
//         Translation2d shotVec = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

//         // 5. CONVERT TO CONTROLS
//         double turretAngle = shotVec.getAngle().getDegrees();
//         double newHorizontalSpeed = shotVec.getNorm();

//         // 6. SOLVE FOR NEW PITCH/RPM
//         // Assuming constant total exit velocity, variable hood:
//         double totalExitVelocity = 15.0; // m/s
//         // Clamp to avoid domain errors if we need more speed than possible
//         double ratio = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
//         double newPitch = Math.acos(ratio);

//         turretAngle = MathUtil.inputModulus(turretAngle, 0, 360);
//         // 7. SET OUTPUTS
//         shooter.getTurret().setGoalAngle(Degrees.of(turretAngle).plus(Degrees.of(28)));
//         // shooter.setRPM(calcRPM(totalExitVelocity));
//         shooter.getHood().setGoalAngle(Rotation2d.fromRadians(newPitch));
//         shooter.getShooterWheel().setTargetVelocity(RotationsPerSecond.of(flywheelSpeedMap.get(distanceRobotToGoal)));
//     }
}