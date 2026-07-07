package frc.utility.io.devices;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class EncoderIOCANcoder implements EncoderIO {
    private final CANcoder encoder;
    private final CANcoderConfiguration config = new CANcoderConfiguration();

    private final StatusSignal<Angle> absolutePosition;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;

    private final Debouncer connectedDebounce = new Debouncer(0.5);

    public EncoderIOCANcoder(int deviceId, CANBus canBus, SensorDirectionValue direction) {
        encoder = new CANcoder(deviceId, canBus);

        config.MagnetSensor.SensorDirection = direction;
        encoder.getConfigurator().apply(config, 0.25);

        absolutePosition = encoder.getAbsolutePosition();
        position = encoder.getPosition();
        velocity = encoder.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, absolutePosition, position, velocity);
        ParentDevice.optimizeBusUtilizationForAll(encoder);
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        var status = BaseStatusSignal.refreshAll(absolutePosition, position, velocity);

        inputs.connected = connectedDebounce.calculate(status.isOK());
        inputs.absolutePositionRotations = absolutePosition.getValueAsDouble();
        inputs.positionRotations = position.getValueAsDouble();
        inputs.velocityRotationsPerSecond = velocity.getValueAsDouble();
    }

    @Override
    public int getDeviceId() {
        return encoder.getDeviceID();
    }

    @Override
    public void resetPosition(double positionRotations) {
        encoder.setPosition(positionRotations);
    }
}
