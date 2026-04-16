package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.DroidRageConstants;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.Dashboard;
import lombok.Getter;
import lombok.Setter;

public class Shooter implements Dashboard, Sendable {
    public enum ShooterValue {
        SHOOT_OUTPOST(71,15,70), //v70
        SHOOT_DEPOT(0,0,0),

        AUTO_SHOOT_DEPOT(SHOOT_DEPOT.getTurretAngle().in(Degrees),SHOOT_DEPOT.getHoodAngle().in(Degrees),0),

        SHORT(180, 10, 45),// 45
        FAR(180,0,0),

        SHOOT_TRENCH_RIGHT(-11,5.57,57.5), //v52.5
        // SHOOT_TRENCH_RIGHT_AP(-11,5.57,57.5), //v52.5

        AUTO_SHOOT_TRENCH_RIGHT_FAR(-15,6.57,63),
        AUTO_SHOOT_TRENCH_LEFT_FAR(15,6.57,63),
    
        // AUTO_SHOOT_TRENCH_RIGHT(SHOOT_TRENCH_RIGHT.getTurretAngle().in(Degrees),
        //     0,
        //     0), 
        // AUTO_SHOOT_TRENCH_RIGHT_TWO(SHOOT_TRENCH_RIGHT.getTurretAngle().in(Degrees),0,
        // SHOOT_TRENCH_RIGHT.getVelocity().in(RotationsPerSecond)), 

        
        SHOOT_TRENCH_LEFT(
            11, 
            SHOOT_TRENCH_RIGHT.getHoodAngle().in(Degrees), 
            SHOOT_TRENCH_RIGHT.getVelocity().in(RotationsPerSecond)
        ),

        // AUTO_SHOOT_TRENCH_LEFT(
        //     SHOOT_TRENCH_LEFT.getTurretAngle().in(Degrees), 
        //     0,
        //     0),
        // AUTO_SHOOT_TRENCH_LEFT_TWO(
        //     SHOOT_TRENCH_LEFT.getTurretAngle().in(Degrees),
        //     0,
        //     SHOOT_TRENCH_LEFT.getVelocity().in(RotationsPerSecond)), 
        
        HOLD(210, 0, 20),//-220
        // HOARD(0,10,60)

        CORNER_RIGHT(
            210, 18, 60
        ),
        CORNER_LEFT(
            148, 
            CORNER_RIGHT.getHoodAngle().in(Degrees),
            CORNER_RIGHT.getVelocity().in(RotationsPerSecond)
        )
        ;
        

        @Getter private final Angle turretAngle;
        @Getter private final Angle hoodAngle;
        @Getter private final AngularVelocity velocity;

        private ShooterValue(double turretAngle, double hoodAngle, double velocity) {
            this.turretAngle = Degrees.of(turretAngle);
            this.hoodAngle = Degrees.of(hoodAngle);
            this.velocity = RotationsPerSecond.of(velocity);
        }
        // private ShooterValue(double turretAngle, double hoodAngle, AngularVelocity velocity) {
        //     this.turretAngle = Degrees.of(turretAngle);
        //     this.hoodAngle = Rotation2d.fromDegrees(hoodAngle);
        //     this.velocity = velocity;
        // }
    }


    public static final Transform2d ROBOT_TO_TURRET_TRANSFORM =
        new Transform2d(
            new Translation2d(Inches.zero(), Inches.of(-9.32)), //-13.25
            new Rotation2d(Degrees.of(0)));//-32.5
    public static final InterpolatingDoubleTreeMap hoodMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap flywheelSpeedMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap timeOfFlightMap =
        new InterpolatingDoubleTreeMap();
    // public static final InterpolatingDoubleTreeMap driveSpeedMap =
    //     new InterpolatingDoubleTreeMap();
    static{

        //Comp
        flywheelSpeedMap.put(1.5,41.5);
        flywheelSpeedMap.put(2.0,42.7);
        flywheelSpeedMap.put(2.9,46.0);
        flywheelSpeedMap.put(4.20,50.0);
        flywheelSpeedMap.put(5.10,53.5);
        flywheelSpeedMap.put(5.6,58.5);//58.1
        flywheelSpeedMap.put(6.1,60.0);

        hoodMap.put(1.5,2.85);
        hoodMap.put(2.00, 7.8);
        hoodMap.put(2.9,10.7);
        hoodMap.put(4.2,16.8);
        hoodMap.put(5.10,18.5);
        hoodMap.put(5.6,19.5);
        hoodMap.put(6.1,20.0);


        // //Home Positions
        // flywheelSpeedMap.put(1.25,40.0);
        // flywheelSpeedMap.put(1.89,42.0);
        // flywheelSpeedMap.put(3.05,47.0);
        // flywheelSpeedMap.put(4.27,50.2);

        // hoodMap.put(1.25,0.0);
        // hoodMap.put(1.89,11.2);
        // hoodMap.put(3.05,12.1);
        // hoodMap.put(4.27,12.75);

        
        //Values for on the fly Dripping Springs
        timeOfFlightMap.put(5.68,1.16);
        timeOfFlightMap.put(4.55,1.12);
        timeOfFlightMap.put(3.15,1.11);
        timeOfFlightMap.put(1.88,1.01);
        timeOfFlightMap.put(1.38,0.9);

        // DR Values for on the fly
        // timeOfFlightMap.put(1.38,0.9); // NOT CALIBRATED
        // timeOfFlightMap.put(1.76, 1.84);
        // timeOfFlightMap.put(3.50, 1.92);
        // timeOfFlightMap.put(5.00, 2.00); // NOT CALIBRATED

        //State Comp...
        // timeOfFlightMap.put(1.38,1.); // NOT CALIBRATED
        // timeOfFlightMap.put(2.6, 0.546);
        // timeOfFlightMap.put(3.4, .647);
        // timeOfFlightMap.put(4.26, .7025); // NOT CALIBRATED


        //Mech Advantage
        // timeOfFlightMap.put(1.63, 1.017);
        // timeOfFlightMap.put(2.40, 0.967);
        // timeOfFlightMap.put(3.25, 1.19);
        // timeOfFlightMap.put(4.15, 1.18);
        // timeOfFlightMap.put(4.875, 1.25);



        // driveSpeedMap.put(1.,1.0);
        // driveSpeedMap.put(3.,2.0);
        // driveSpeedMap.put(5.,3.0);
        // driveSpeedMap.put(7.,4.0);
        // driveSpeedMap.put(10.,5.0);

        //M/s to Multiplier 
        // driveSpeedMap.put(0.1,1.05);
        // driveSpeedMap.put(1.0,1.3);
        // driveSpeedMap.put(1.5,1.6);



        


    }

    @Getter private final Turret turret;
    @Getter private final Hood hood;
    @Getter private final ShooterWheel shooterWheel;
    
    public static final AngularVelocity IDLE_VELOCITY = RotationsPerSecond.of(30);

    @Getter @Setter private ShooterValue currentShooterPos = ShooterValue.HOLD;

    public Shooter (
        Turret turret,
        Hood hood,
        ShooterWheel shooter
    ) {
        this.turret=turret;
        this.hood=hood;
        this.shooterWheel=shooter;

        shooterWheel.setTargetVelocity(IDLE_VELOCITY);

        TelemetryUtils.registerDashboard(this);

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // builder.addStringProperty("ShooterMode", () -> getCurrentShooterPos().toString(), null);
        builder.addBooleanProperty("Is Shooter Ready?", this::isShooterReady, null);
        builder.addBooleanProperty("Is Hood Down", ()-> hood.getCurrentAngle().in(Degree) < 2, null);

    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData("Shooter", this);
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    public Command setShooterTargetCommand(ShooterValue shooterValue) {
        return new ParallelCommandGroup(
            // Commands.runOnce(()->setCurrentShooterPos(shooterValue)),
            new InstantCommand(()-> DroidRageConstants.isShooterManual = true),
            hood.setTargetPositionCommand(shooterValue.getHoodAngle()),
            turret.setTargetPositionCommand(shooterValue.getTurretAngle()),
            shooterWheel.setTargetVelocityCommand(shooterValue.getVelocity())
        );
    }

    public void setShooterTarget(ShooterValue shooterValue) {
        hood.setGoalAngle(shooterValue.getHoodAngle());
        turret.setGoalAngle(shooterValue.getTurretAngle());
        shooterWheel.setTargetVelocity(shooterValue.getVelocity());
    }

    public Command setHoodPositionCommand(ShooterValue shooterValue) {
        return hood.setTargetPositionCommand(shooterValue.getHoodAngle());
    }

    public boolean isShooterReady() {
        return (
            hood.getPositionError().in(Degrees) < 5 && 
            turret.getPositionError().in(Degrees) < 5 &&
            hood.getPositionError().in(Degrees) > -5 && 
            turret.getPositionError().in(Degrees) > -5
        );
    }

    public static double getDistanceToHub(Pose2d robotPose, Translation2d target){
        System.out.println("Distance:" + target.getDistance(robotPose.getTranslation()));
        return target.getDistance(robotPose.getTranslation());
    }

    public static Pose2d predictPosePos(Pose2d currentPose, Translation2d goalPose, ChassisSpeeds fieldSpeeds, double distanceRobotToGoal) {
        // System.out.println("X:" + fieldSpeeds.vxMetersPerSecond);
        // System.out.println("Y:" + fieldSpeeds.vyMetersPerSecond);
        // double num = Math.sqrt(Math.pow(fieldSpeeds.vxMetersPerSecond, 2) + Math.pow(fieldSpeeds.vyMetersPerSecond, 2));
        // System.out.println("B:" + num);

        // double multiplier = Shooter.driveSpeedMap.get(num); // Lookahead time in seconds
        // double predictedX = currentPose.getX() + fieldSpeeds.vxMetersPerSecond * Shooter.driveSpeedMap.get(Math.abs(fieldSpeeds.vxMetersPerSecond));
        // double predictedY = currentPose.getY() + fieldSpeeds.vyMetersPerSecond * Shooter.driveSpeedMap.get(Math.abs(fieldSpeeds.vyMetersPerSecond));
        
        
        double predictedX = currentPose.getX() + fieldSpeeds.vxMetersPerSecond * timeOfFlightMap.get(distanceRobotToGoal);
        double predictedY = currentPose.getY() + fieldSpeeds.vyMetersPerSecond * timeOfFlightMap.get(distanceRobotToGoal);
        
        // double predictedX = currentPose.getX() + fieldSpeeds.vxMetersPerSecond;
        // double predictedY = currentPose.getY() + fieldSpeeds.vyMetersPerSecond;
        return new Pose2d(predictedX, predictedY, currentPose.getRotation());
    }

    public static Translation2d predictPosePosMoveHub(Pose2d currentPose, Translation2d goalPose, ChassisSpeeds fieldSpeeds, double distanceRobotToGoal) {
    
        double newGoalX = goalPose.getX() + fieldSpeeds.vxMetersPerSecond * timeOfFlightMap.get(distanceRobotToGoal);
        double newGoalY = goalPose.getY() + fieldSpeeds.vyMetersPerSecond * timeOfFlightMap.get(distanceRobotToGoal);
        
        return new Translation2d(newGoalX, newGoalY);
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
}
