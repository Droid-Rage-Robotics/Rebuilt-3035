package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.shooter.HubShooterMath.ShotData;
import frc.robot.subsystems.vision.Vision;
import frc.utility.LimelightEx;
import frc.utility.TelemetryUtils.Dashboard;
import lombok.Getter;
import lombok.Setter;

public class Shooter implements Dashboard, Sendable{
    public enum ShooterMode {
        HOLD, //Maintain current position
        OPPOSITE, //Face opposite intake
        SCORE, //Hub Scoring
        HOARD
    }
    private enum TargetHeight {
        TAG_2(2, 0),
        TAG_3(3, 0),
        TAG_4(4, 0),
        TAG_5(5, 0),
        TAG_8(8, 0),
        TAG_9(9, 0),
        TAG_10(10, 0),
        TAG_11(11, 0),
        TAG_18(18, 0),
        TAG_19(19, 0),
        TAG_20(20, 0),
        TAG_21(21, 0),
        TAG_24(24, 0),
        TAG_25(25, 0),
        TAG_26(26, 0),
        TAG_27(27, 0);

        private final int id;
        private final double height;

        private TargetHeight(int id, double height) {
            this.id = id;
            this.height = height;
        }

        public static double getHeight(double tagId) {
            for (TargetHeight target : values()) {
                if (target.id == tagId) {
                    return target.height;
                }
            }
            return 0; // Default height if tag ID not found - TODO: Fix
        }
    }
    
    private static final InterpolatingDoubleTreeMap SCORE_SPEED_MAP = new InterpolatingDoubleTreeMap();
    // private static final InterpolatingDoubleTreeMap HOARD_SPEED_MAP = new InterpolatingDoubleTreeMap();

    static{
        //Include all speed values for the shooter (LL Distance, Speed)

        //Scoring
        SCORE_SPEED_MAP.put(0.0,0.0);
        SCORE_SPEED_MAP.put(4.1,100.0);


        // //Hoarding
        // HOARD_SPEED_MAP.put(0.0,0.0);
        // HOARD_SPEED_MAP.put(4.1,100.0);

    }

    private final Turret turret;
    private final Hood hood;
    private final ShooterWheel shooter;
    private final Vision vision;
    
    private final double IDLE_RPM = 0;
    private final double OPP_ANGLE = 0;

    private static final double LIMELIGHT_HEIGHT=0;
    private static final double LIMELIGHT_PITCH=0;

    private static final Distance SHOOTER_WHEEL_RADIUS = Inches.of(0);
    private static final double SHOOTER_EFFICIENCY = 1;

    private final NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("Drivetrain");

    private final StructSubscriber<Rotation2d> yawSubscriber = driveTable.getStructTopic("Yaw", Rotation2d.struct).subscribe(new Rotation2d());
    private final StructSubscriber<Pose3d> pose3dSub = driveTable.getStructTopic("Pose3d", Pose3d.struct).subscribe(new Pose3d());
    private final StructSubscriber<Pose2d> poseSub = driveTable.getStructTopic("Pose2d", Pose2d.struct).subscribe(new Pose2d());
    private final StructSubscriber<ChassisSpeeds> chassisSpeedsSub = driveTable.getStructTopic("ChassisSpeeds", ChassisSpeeds.struct).subscribe(new ChassisSpeeds());




    private final LimelightEx limelight;

    @Getter @Setter private ShooterMode shooterMode;

    public Shooter (
        Turret turret,
        Hood hood,
        ShooterWheel shooter,
        Vision vision
    ) {
        this.turret=turret;
        this.hood=hood;
        this.shooter=shooter;
        this.vision=vision;

        limelight = vision.getTurretLL();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("ShooterMode", () -> getShooterMode().toString(), null);
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData("Shooter", this);
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    public Command setShooterModeCommand(ShooterMode shooterMode) {
        return new InstantCommand(() -> setShooterMode(shooterMode));
    }
    
    public void turret() {
        switch(shooterMode){
            case HOLD:
                turret.setTargetPositionCommand(turret.getPositionSetpoint());
                hood.setTargetPositionCommand(hood.getPositionSetpoint());
                shooter.setTargetVelocityCommand(shooter.getTargetVelocity());
                shooter.setTargetVelocityCommand(IDLE_RPM);

                //Should shooter be at same speed or just stop?
                break;
            case OPPOSITE:
                turret.setTargetPositionCommand(OPP_ANGLE);
                getShooterSpeed(); //Based on limelight
                break;
            case SCORE:
                turretAim();
                break;
            case HOARD:
                hoardAiming();
                break;
        }
    }

    public void getShooterSpeed(){
        shooter.setTargetVelocityCommand(SCORE_SPEED_MAP.get(limelight.getTY()));
    }
    
    public void aim() {
        if (limelight.getTV()) {
            ShotData shot = HubShooterMath.iterativeMovingShotFromFunnelClearance(
                poseSub.get(), 
                chassisSpeedsSub.get(), 
                limelight.getTargetPose_FieldSpace().getTranslation(), 
                1
            );

            hood.setGoalAngle(new Rotation2d(shot.getHoodAngle()));
        }

        // var tof = Einstein.calculateTimeOfFlight(getShooterExitVelocity(), null, 0);

        // var tar = Einstein.predictTargetPos(null, null, null);

        // var exit = Einstein.calculateShotFromFunnelClearance(null, null, null);

        // var angle = Einstein.calculateAngleFromVelocity(null, getShooterExitVelocity(), null);


        /* ---------------- Turret Aiming ---------------- */
        
        
    }

    private void turretAim() {
        if (limelight.getTV()) {
            double txDeg = limelight.getTX();
            Rotation2d currentAngle = turret.getCurrentAngle();

            // Shift the goal by the Limelight error
            Rotation2d newGoal = currentAngle.plus(Rotation2d.fromDegrees(txDeg));
            turret.setGoalAngle(newGoal);
        }
    }

    private void hoardAiming() {
         // 1. Determine target field angle based on alliance
        Rotation2d targetFieldAngle;
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            targetFieldAngle = Rotation2d.fromDegrees(0.0); // Point toward red
        } else {
            targetFieldAngle = Rotation2d.fromDegrees(180.0); // Point toward blue
        }

        // 2. Get current robot heading from Pigeon 2
        Rotation2d robotHeading = yawSubscriber.get();

        // 3. Convert target field angle to robot-relative
        Rotation2d robotRelativeAngle = targetFieldAngle.minus(robotHeading);

        // 4. Account for turret's 180° offset
        Rotation2d turretSetpoint = robotRelativeAngle.minus(Rotation2d.fromDegrees(180.0));

        // 5. Send to turret (Rotation2d handles normalization automatically)
        turret.setGoalAngle(turretSetpoint);
    }
}
