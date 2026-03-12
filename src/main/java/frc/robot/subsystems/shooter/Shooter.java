package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DroidRageConstants;
import frc.robot.DroidRageConstants.FieldConstants;
import frc.robot.commands.shooter.ShootHub;
import frc.robot.subsystems.shooter.HubShooterMath.ShotData;
import frc.utility.TelemetryUtils.Dashboard;
import frc.utility.TelemetryUtils.Periodic;
import lombok.Getter;
import lombok.Setter;

public class Shooter implements Dashboard, Sendable, Periodic {
    public enum ShooterMode {
        HOLD, //Maintain current position
        OPPOSITE, //Face opposite intake
        SCORE, //Hub Scoring
        HOARD // shooting on alliance side
    }
    public enum ShooterValue {
        // SHOOT_HUB(180,15,70),
        SHOOT_OUTPOST(180,15,70),

        SHORT(0, 10, 52),
        FAR(0,0,0),

        SHOOT_TRENCH_RIGHT(138.5,5.57,52),
        // AUTO_SHOOT_TRENCH_RIGHT(
        //     SHOOT_TRENCH_RIGHT.getTurretAngle().getDegrees(),
        //     0,
        //     SHOOT_TRENCH_RIGHT.getVelocity()),
        SHOOT_TRENCH_LEFT(
            -138.5, 
            SHOOT_TRENCH_RIGHT.getHoodAngle().getDegrees(), 
            SHOOT_TRENCH_RIGHT.getVelocity()),
        // AUTO_SHOOT_TRENCH_LEFT(
        //     SHOOT_TRENCH_LEFT.getTurretAngle().getDegrees(), 
        //     AUTO_SHOOT_TRENCH_RIGHT.getHoodAngle().getDegrees(),
        //     AUTO_SHOOT_TRENCH_RIGHT.getVelocity()),
        
        HOLD(-220, 0, 20),
        HOARD(0,5,40)
        ;

        @Getter private final Rotation2d turretAngle;
        @Getter private final Rotation2d hoodAngle;
        @Getter private final AngularVelocity velocity;

        private ShooterValue(double turretAngle, double hoodAngle, double velocity) {
            this.turretAngle = Rotation2d.fromDegrees(turretAngle);
            this.hoodAngle = Rotation2d.fromDegrees(hoodAngle);
            this.velocity = RotationsPerSecond.of(velocity);
        }
        private ShooterValue(double turretAngle, double hoodAngle, AngularVelocity velocity) {
            this.turretAngle = Rotation2d.fromDegrees(turretAngle);
            this.hoodAngle = Rotation2d.fromDegrees(hoodAngle);
            this.velocity = velocity;
        }
    }

    @Getter private final Turret turret;
    @Getter private final Hood hood;
    @Getter private final ShooterWheel shooterWheel;
    
    public static final AngularVelocity IDLE_VELOCITY = RotationsPerSecond.of(30);
    // private final double OPP_ANGLE = 0;

    // private static final double LIMELIGHT_HEIGHT=0;
    // private static final double LIMELIGHT_PITCH=0;
    // private static final double SHOOTER_EFFICIENCY = 1;

    private final NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("DriveState");
    private final StructSubscriber<Pose2d> poseSub = driveTable.getStructTopic("Pose", Pose2d.struct).subscribe(new Pose2d());
    private final StructSubscriber<ChassisSpeeds> chassisSpeedsSub = driveTable.getStructTopic("Speeds", ChassisSpeeds.struct).subscribe(new ChassisSpeeds());

    @Getter @Setter private ShooterValue shooterValue;

    public Shooter (
        Turret turret,
        Hood hood,
        ShooterWheel shooter
    ) {
        this.turret=turret;
        this.hood=hood;
        this.shooterWheel=shooter;

        shooterValue=ShooterValue.HOLD;
    }

    public void periodic() {
        
        // ShotData shot = HubShooterMath.iterativeMovingShotFromFunnelClearance(
        //     poseSub.get(), 
        //     chassisSpeedsSub.get(), 
        //     FieldConstants.HUB_BLUE, 
        //     1
        // );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // builder.addStringProperty("ShooterMode", () -> getShooterMode().toString(), null);
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData("Shooter", this);
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    // public void shooterPeriodic() {
    //     switch(shooterMode){
    //         case HOLD:
    //             break;
    //         case OPPOSITE:
    //         case HOARD:
    //             break;
    //         case SCORE:
    //             break;
    //         default:
    //             break;
    //     }
    // }

    // public Command setShooterModeCommand(ShooterMode shooterMode) {
    //     return new InstantCommand(() -> setShooterMode(shooterMode));
    // }

    public Command setShooterTargetCommand(ShooterValue shooterValue) {
        return new ParallelCommandGroup(
            new InstantCommand(()-> setShooterValue(shooterValue)),
            turret.setTargetPositionCommand(shooterValue.getTurretAngle()),
            shooterWheel.setTargetVelocityCommand(shooterValue.getVelocity())
        );
    }

    public Command raiseHoodCommand(Shooter shooter) {
        return hood.setTargetPositionCommand(shooterValue.getHoodAngle());
    }

    public Command lowerHoodCommand(Shooter shooter) {
        return hood.setTargetPositionCommand(Rotation2d.kZero);
    }
}
