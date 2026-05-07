package frc.utility.io;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.utility.devices.encoder.CANcoderEx;
import frc.utility.devices.encoder.EncoderConstants;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.io.devices.MotorIO;
import frc.utility.io.devices.MotorIOTalonFX;
import frc.utility.template.Constants.EncoderType;
import frc.utility.template.Constants.TurretConstants;

public class TurretIOTalonFX implements TurretIO {
    private final MotorIOTalonFX motorIO;
    private final MotorIO.MotorIOInputs motorInputs = new MotorIO.MotorIOInputs();

    private final TalonFX motor;
    private final Optional<CANcoderEx> encoder;

    private final TurretConstants constants;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    private final StatusSignal<Double> closedLoopReference;
    private final StatusSignal<Double> closedLoopReferenceSlope;
    private final StatusSignal<Double> closedLoopError;

    private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);

    private boolean isEnabled;

    public TurretIOTalonFX(
            boolean isEnabled,
            TurretConstants constants,
            EncoderConstants encoderConstants,
            MotorConstants motorConstants
    ) {
        this.constants = constants;
        this.isEnabled = isEnabled;

        motorConstants.isEnabled = isEnabled;

        motorIO = new MotorIOTalonFX(motorConstants);
        motor = motorIO.getMotor();

        var config = motorConstants.getConfig();

        if (constants.encoderType == EncoderType.ABSOLUTE
                || constants.encoderType == EncoderType.EXTERNAL) {
            if (encoderConstants == null) {
                throw new NullPointerException("Encoder constants required for external encoder");
            }

            encoder = Optional.of(CANcoderEx.createWithConstants(encoderConstants));

            config.Feedback.FeedbackRemoteSensorID = encoderConstants.deviceId;
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        } else {
            encoder = Optional.empty();
        }

        config.Feedback.SensorToMechanismRatio = constants.gearRatio;

        config.Slot0.kP = constants.kP;
        config.Slot0.kI = constants.kI;
        config.Slot0.kD = constants.kD;
        config.Slot0.kV = constants.kV;
        config.Slot0.kA = constants.kA;
        config.Slot0.kS = constants.kS;

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
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        motorIO.updateInputs(motorInputs);

        var closedLoopStatus = BaseStatusSignal.refreshAll(
            closedLoopReference,
            closedLoopReferenceSlope,
            closedLoopError
        );

        inputs.motorConnected = motorInputs.connected;
        inputs.encoderConnected = encoderConnectedDebounce.calculate(closedLoopStatus.isOK());

        inputs.position = motorInputs.position;
        inputs.velocity = motorInputs.velocity;
        inputs.appliedVoltage = motorInputs.appliedVolts;
        inputs.statorCurrent = motorInputs.statorCurrent;

        inputs.closedLoopReference =
            Rotations.of(closedLoopReference.getValueAsDouble());

        inputs.closedLoopReferenceVelocity =
            RotationsPerSecond.of(closedLoopReferenceSlope.getValueAsDouble());

        inputs.closedLoopError =
            Rotations.of(closedLoopError.getValueAsDouble());
    }

    @Override
    public void setPosition(Angle angle) {
        if (isEnabled) {
            motorIO.setControl(motionMagicRequest.withPosition(angle));
        }
    }

    @Override
    public void setVoltage(Voltage voltage) {
        if (isEnabled) {
            motorIO.setControl(voltageRequest.withOutput(voltage));
        }
    }

    @Override
    public void resetEncoder(Angle angle) {
        switch (constants.encoderType) {
            case ABSOLUTE:
                return;

            case EXTERNAL:
                encoder.get().resetPosition(angle);
                motorIO.setPosition(angle);
                break;

            case INTEGRATED:
                motorIO.setPosition(angle);
                break;
        }
    }
}