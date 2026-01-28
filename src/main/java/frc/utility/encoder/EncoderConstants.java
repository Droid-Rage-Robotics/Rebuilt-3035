package frc.utility.encoder;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class EncoderConstants {
    private final CANcoderConfiguration config = new CANcoderConfiguration();

    public Integer deviceId;
    public CANBus canBus;
    public double conversionFactor = 1;

    /**
     * Used to set the canbus of the motor.
     * This method may only be called once.
     * @param canBus the canbus the motor is on
     * @return EncoderConstants (for call chaining)
     * @throws IllegalStateException Value has already been set!
     */
    public EncoderConstants withCANBus(CANBus canBus) {
        if (this.canBus != null) {
            throw new IllegalStateException("Value has already been set");
        } else {
            this.canBus=canBus;
        }
        return this;
    }

    /**
     * Used to set the deviceid of the motor
     * This method may only be called once.
     * @param deviceid the deviceid of the motor
     * @return EncoderConstants (for call chaining)
     * @throws IllegalStateException Value has already been set!
     */
    public EncoderConstants withDeviceId(int deviceId) {
        if (this.deviceId != null) {
            throw new IllegalStateException("Value has already been set");
        } else {
            this.deviceId=deviceId;
        }
        return this;
    }

    /**
     * Used to set a conversion factor to be applied to
     * the raw position/velocity of the motor. Defaults
     * to 1.
     * @param conversionFactor
     * @return EncoderConstants (for call chaining)
     */
    public EncoderConstants withConversionFactor(double conversionFactor) {
        this.conversionFactor=conversionFactor;
        return this;
    }

    /**
     * Modifies this configuration's SensorDirection parameter and returns itself for
     * method-chaining and easier to use config API.
     * <p>
     * Direction of the sensor to determine positive rotation, as seen
     * facing the LED side of the CANcoder.
     * 
     *
     * @param newSensorDirection Parameter to modify
     * @return EncoderConstants (for call chaining)
     */
    public EncoderConstants withDirection(SensorDirectionValue value) {
        config.MagnetSensor.SensorDirection=value;
        return this;
    }

    public EncoderConstants wihZeroOffset(double offset) {
        config.MagnetSensor.MagnetOffset=offset;
        return this;
    }

    public CANcoderConfiguration getConfig() {
        return config;
    }
}
