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
    public static final Transform3d ROBOT_TO_TURRET_TRANSFORM =
        new Transform3d(new Translation3d(Inches.zero(), Inches.of(-13.25), Inches.of(6.613)), 
            new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));//-32.5
    private static final InterpolatingDoubleTreeMap hoodMap =
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
        new InterpolatingDoubleTreeMap();
    static{
        //Springs
        // flywheelSpeedMap.put(1.25,40.0);
        // flywheelSpeedMap.put(1.89,42.0);
        // flywheelSpeedMap.put(3.05,47.0);
        // flywheelSpeedMap.put(4.27,50.2);

        // hoodMap.put(1.25,.0);
        // hoodMap.put(1.89,11.2);
        // hoodMap.put(3.05,12.1);
        // hoodMap.put(4.27,12.75);



        //Home Positions
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

        if(DroidRageConstants.alliance == Alliance.Red){
            this.hubPose = FieldConstants.HUB_RED;
        } else{// if (DroidRageConstants.alliance == Alliance.Blue){
            this.hubPose = FieldConstants.HUB_BLUE;

        }
        // this.hubPose = DroidRageConstants.alliance == Alliance.Red 
        //     ? FieldConstants.HUB_RED 
        //     : FieldConstants.HUB_BLUE;
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
        if(DroidRageConstants.alliance == Alliance.Red){
            this.hubPose = FieldConstants.HUB_RED;
        } else if (DroidRageConstants.alliance == Alliance.Blue){
            this.hubPose = FieldConstants.HUB_BLUE;

        }
            // this.hubPose = FieldConstants.HUB_RED;

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
        System.out.println(distanceRobotToGoal);
        // SmartDashboard.putNumber("Shooter/Goal Distance", distanceRobotToGoal);

        if (!DroidRageConstants.isShooterManual) {
            switch(DRAreaManager.getCurrentZone()){
                case ALLIANCE_ZONE,NEUTRAL,OPPOSITION:
                    shooter.getTurret().setGoalAngle(calculateAzimuthAngle(drive.getState().Pose, hubPose));
                    // System.out.println(calculateAzimuthAngle(drive.getState().Pose, hubPose).in(Degrees));

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
    
    public static double getDistanceToHub(Pose2d robotPose, Translation2d target){
        // Translation2d target2d = new Translation2d(target.getX(), target.getY());
        return target.getDistance(robotPose.getTranslation());
        // return robotPose.getTranslation().getDistance(target2d);
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
    public static Angle calculateAzimuthAngle(Pose2d robot, Translation2d target) {
        Translation2d turretTranslation = new Pose3d(robot)
                .transformBy(ROBOT_TO_TURRET_TRANSFORM)
                .toPose2d()
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