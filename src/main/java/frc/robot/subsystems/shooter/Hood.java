package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Hood extends SubsystemBase {
    private final TalonFX hoodMotor = new TalonFX(1);
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
        .withExternalEncoder(new CANcoder(0))
        .withExternalEncoderInverted(true)
        .withExternalEncoderGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withExternalEncoderZeroOffset(Degrees.of(0))
        .withUseExternalFeedbackEncoder(true)
        .withSoftLimit(Degrees.of(-30), Degrees.of(100))
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))
        .withControlMode(ControlMode.CLOSED_LOOP);
    private final SmartMotorController smartMotorController = new TalonFXWrapper(hoodMotor,DCMotor.getKrakenX60(1), motorConfig);
    private final ArmConfig armCfg = new ArmConfig(smartMotorController)
        .withLength(Feet.of(3))
        .withMass(Pounds.of(1));

    private final Arm hood = new Arm(armCfg);

     
}
