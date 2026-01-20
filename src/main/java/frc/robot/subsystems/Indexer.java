package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Indexer extends SubsystemBase{
    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        // Telemetry name and verbosity level
        .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));

    // Vendor motor controller object
    private final TalonFX motor = new TalonFX(4);

    // Create our SmartMotorController from our Spark and config with the NEO.
    private final SmartMotorController talonSmartMotorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), smcConfig);

    private final FlyWheelConfig indexerConfig = new FlyWheelConfig()
        // Diameter of the flywheel.
        .withDiameter(Inches.of(4))
        // Mass of the flywheel.
        .withMass(Pounds.of(1))
        // Maximum speed of the indexer.
        .withUpperSoftLimit(RPM.of(1000))
        // Telemetry name and verbosity for the arm.
        .withTelemetry("Indexer", TelemetryVerbosity.HIGH);

    // Indexer Mechanism
    private final FlyWheel indexer = new FlyWheel(indexerConfig);
}
