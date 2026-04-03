package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
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

        SHORT(180, 10, 45),// 45
        FAR(180,0,0),

        SHOOT_TRENCH_RIGHT(-15,5.57,47.5), //v52.5

        AUTO_SHOOT_TRENCH_RIGHT(SHOOT_TRENCH_RIGHT.getTurretAngle().in(Degrees),0,0), //v52
        
        SHOOT_TRENCH_LEFT(
            15, 
            SHOOT_TRENCH_RIGHT.getHoodAngle().getDegrees(), 
            SHOOT_TRENCH_RIGHT.getVelocity()
        ),

        AUTO_SHOOT_TRENCH_LEFT(SHOOT_TRENCH_LEFT.getTurretAngle().in(Degrees), 0,0),
        
        HOLD(210, 0, 20),//-220
        // HOARD(0,10,60)

        CORNER_RIGHT(
            210, 18, 60
        ),
        CORNER_LEFT(
            148, 
            CORNER_RIGHT.getHoodAngle().getDegrees(),
            CORNER_RIGHT.getVelocity()
        )
        ;
        

        @Getter private final Angle turretAngle;
        @Getter private final Rotation2d hoodAngle;
        @Getter private final AngularVelocity velocity;

        private ShooterValue(double turretAngle, double hoodAngle, double velocity) {
            this.turretAngle = Degrees.of(turretAngle);
            this.hoodAngle = Rotation2d.fromDegrees(hoodAngle);
            this.velocity = RotationsPerSecond.of(velocity);
        }
        private ShooterValue(double turretAngle, double hoodAngle, AngularVelocity velocity) {
            this.turretAngle = Degrees.of(turretAngle);
            this.hoodAngle = Rotation2d.fromDegrees(hoodAngle);
            this.velocity = velocity;
        }
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
        TelemetryUtils.registerDashboard(this);

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("ShooterMode", () -> getCurrentShooterPos().toString(), null);
        builder.addBooleanProperty("Is Shooter Ready?", this::isShooterReady, null);
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
            hood.getSetpointError().in(Degrees) < 5 && 
            turret.getPositionError().in(Degrees) < 5 &&
            hood.getSetpointError().in(Degrees) > -5 && 
            turret.getPositionError().in(Degrees) > -5
        );
    }
    
}
