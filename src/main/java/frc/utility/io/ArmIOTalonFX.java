package frc.utility.io;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.io.devices.EncoderIO;
import frc.utility.io.devices.EncoderIOCANcoder;
import frc.utility.io.devices.EncoderIOInputsAutoLogged;
import frc.utility.io.devices.MotorIO;
import frc.utility.io.devices.MotorIOTalonFX;
import frc.utility.template.Constants.ArmConstants;
import frc.utility.template.Constants.EncoderType;

public class ArmIOTalonFX implements ArmIO {
    private final MotorIOTalonFX motorIO;
    private final MotorIO.MotorIOInputs motorInputs = new MotorIO.MotorIOInputs();
    private final EncoderIOInputsAutoLogged encoderInputs = new EncoderIOInputsAutoLogged();

    private final TalonFX motor;
    private final Optional<EncoderIO> encoderIO;
    private final Optional<EncoderIOCANcoder> batchedEncoder;
    private final BaseStatusSignal[][] refreshGroups;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);

    private final ArmConstants constants;

    private final StatusSignal<Double> closedLoopReference;
    private final StatusSignal<Double> closedLoopReferenceSlope;
    private final StatusSignal<Double> closedLoopError;

    private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);
    private static final double RADIANS_PER_ROTATION = 2.0 * Math.PI;

    public ArmIOTalonFX(
            boolean isEnabled,
            ArmConstants constants,
            EncoderIO encoderIO,
            MotorConstants motorConstants
    ) {
        this.constants = constants;

        motorConstants.isEnabled = isEnabled;

        motorIO = new MotorIOTalonFX(motorConstants);
        motor = motorIO.getMotor();

        var config = motorIO.getConfig();

        if (constants.encoderType == EncoderType.ABSOLUTE
                || constants.encoderType == EncoderType.EXTERNAL) {
            if (encoderIO == null) {
                throw new NullPointerException("Encoder IO required for external encoder");
            }

            this.encoderIO = Optional.of(encoderIO);

            config.Feedback.FeedbackRemoteSensorID = encoderIO.getDeviceId();
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        } else {
            this.encoderIO = Optional.empty();
        }

        config.Feedback.SensorToMechanismRatio = constants.gearRatio;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Slot0.kP = constants.kP;
        config.Slot0.kI = constants.kI;
        config.Slot0.kD = constants.kD;
        config.Slot0.kV = constants.kV;
        config.Slot0.kA = constants.kA;
        config.Slot0.kS = constants.kS;
        config.Slot0.kG = constants.kG;

        config.MotionMagic.MotionMagicCruiseVelocity =
                constants.maxVelocity.in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration =
                constants.maxAcceleration.in(RotationsPerSecondPerSecond);
        config.MotionMagic.MotionMagicJerk = constants.maxJerk;

        motor.getConfigurator().apply(config, 0.25);

        closedLoopReference = motor.getClosedLoopReference();
        closedLoopReferenceSlope = motor.getClosedLoopReferenceSlope();
        closedLoopError = motor.getClosedLoopError();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                closedLoopReference,
                closedLoopReferenceSlope,
                closedLoopError
        );

        ParentDevice.optimizeBusUtilizationForAll(motor);

        // Keep non-Phoenix encoder implementations on their normal update path.
        batchedEncoder = this.encoderIO
            .filter(io -> io instanceof EncoderIOCANcoder)
            .map(io -> (EncoderIOCANcoder) io);

        var signalsByNetwork = new LinkedHashMap<String, List<BaseStatusSignal>>();
        var motorSignals = new ArrayList<BaseStatusSignal>();
        Collections.addAll(motorSignals, motorIO.getStatusSignals());
        motorSignals.add(closedLoopReference);
        motorSignals.add(closedLoopReferenceSlope);
        motorSignals.add(closedLoopError);
        signalsByNetwork.put(motorConstants.canBus.getName(), motorSignals);
        batchedEncoder.ifPresent(io -> {
            var signals = signalsByNetwork.computeIfAbsent(
                io.getCANNetworkName(), network -> new ArrayList<>());
            Collections.addAll(signals, io.getStatusSignals());
        });
        refreshGroups = signalsByNetwork.values().stream()
            .map(signals -> signals.toArray(new BaseStatusSignal[0]))
            .toArray(BaseStatusSignal[][]::new);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        for (var signals : refreshGroups) {
            BaseStatusSignal.refreshAll(signals);
        }
        motorIO.copyInputs(motorInputs);
        if (batchedEncoder.isPresent()) {
            batchedEncoder.get().copyInputs(encoderInputs);
        } else {
            encoderIO.ifPresent(io -> io.updateInputs(encoderInputs));
        }

        boolean closedLoopConnected = closedLoopReference.getStatus().isOK()
            && closedLoopReferenceSlope.getStatus().isOK()
            && closedLoopError.getStatus().isOK();

        inputs.motorConnected = motorInputs.connected;

        inputs.positionRad = motorInputs.positionRotations * RADIANS_PER_ROTATION;
        inputs.velocityRadPerSec = motorInputs.velocityRotationsPerSecond * RADIANS_PER_ROTATION;
        inputs.appliedVolts = motorInputs.appliedVolts;
        inputs.statorCurrentAmps = motorInputs.statorCurrentAmps;
        inputs.tempCelsius = motorInputs.tempCelsius;

        inputs.closedLoopReferenceRad =
                closedLoopReference.getValueAsDouble() * RADIANS_PER_ROTATION;

        inputs.closedLoopReferenceVelocityRadPerSec =
                closedLoopReferenceSlope.getValueAsDouble() * RADIANS_PER_ROTATION;

        inputs.closedLoopErrorRad =
                closedLoopError.getValueAsDouble() * RADIANS_PER_ROTATION;

        inputs.encoderConnected =
                encoderConnectedDebounce.calculate(
                        encoderIO.isEmpty() || (encoderInputs.connected && closedLoopConnected));
    }

    @Override
    public void setEnabled(boolean isEnabled) {
        motorIO.setEnabled(isEnabled);
    }

    @Override
    public void setGoalAngle(Angle angle) {
        setGoalAngleRad(angle.in(Radians));
    }

    @Override
    public void setGoalAngleRad(double angleRad) {
        motorIO.setControl(motionMagicRequest.withPosition(angleRad / RADIANS_PER_ROTATION));
    }

    @Override
    public void setVoltage(double volts) {
        motorIO.setVoltage(volts);
    }

    @Override
    public void setVoltage(Voltage volts) {
        motorIO.setVoltage(volts);
    }

    @Override
    public void resetEncoder(Angle angle) {
        switch (constants.encoderType) {
            case ABSOLUTE:
                return;

            case EXTERNAL:
                encoderIO.get().resetPosition(angle);
                motorIO.setPosition(angle);
                break;

            case INTEGRATED:
                motorIO.setPosition(angle);
                break;
        }
    }

    @Override
    public void stop() {
        motorIO.stop();
    }
}
