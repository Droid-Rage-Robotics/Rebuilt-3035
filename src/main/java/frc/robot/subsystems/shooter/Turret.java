package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.utility.LimelightEx;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Turret implements Subsystem {
    private final TalonFX turretMotor = new TalonFX(1);
    private final LimelightEx limelight = LimelightEx.create("turret");
    
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        // Configure Motor and Mechanism properties
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withIdleMode(MotorMode.BRAKE)
        .withMotorInverted(false)
        // Setup Telemetry
        .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
        // Power Optimization
        .withStatorCurrentLimit(Amps.of(40))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));

    private final SmartMotorController motor = new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), motorConfig);

    private final PivotConfig m_config = new PivotConfig(motor)
        .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
        .withWrapping(Degrees.of(0), Degrees.of(360)) // Wrapping enabled bc the pivot can spin infinitely
        .withHardLimit(Degrees.of(0), Degrees.of(720)) // Hard limit bc wiring prevents infinite spinning
        .withTelemetry("PivotExample", TelemetryVerbosity.HIGH) // Telemetry
        .withMOI(Meters.of(0.25), Pounds.of(4)); // MOI Calculation


    private final Pivot turret = new Pivot(m_config);

    public Turret() {}

    public Command setAngle(Angle angle) {
        return turret.setAngle(angle);
    }

    
    public Command aimWithVision() {
        return turret.setAngle(() -> {
            if (!limelight.getTV()) { // target not visible
                return turret.getAngle(); // stay put
            }
            double currentAngleDeg = turret.getAngle().in(Degrees);
            double offsetDeg = limelight.getTX(); // Limelight horizontal error
            double targetDeg = currentAngleDeg + offsetDeg;

            // Clamp to turret limits
            targetDeg = Math.max(0, Math.min(720, targetDeg)); // replace with your hard limits
            return Degrees.of(targetDeg);
        });
    }
    
}