package frc.utility.devices.encoder;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Angle;
import frc.robot.DroidRageConstants;

public class CANcoderEx {
    private final CANcoder encoder;
    private final CANcoderConfiguration config;
    private final int deviceId;

    private CANcoderEx(int deviceId, CANBus canBus) {
        this.deviceId=deviceId;
        this.encoder = new CANcoder(deviceId, canBus);
        this.config = new CANcoderConfiguration();
    }

    private CANcoderEx(EncoderConstants constants) {
        this.deviceId = constants.deviceId;
        this.encoder = new CANcoder(constants.deviceId, constants.canBus);
        this.config = constants.getConfig();
        withConfiguration(config);
    }

    /**
     * Creates a new CANcoderEx instance with the specified
     * device id and canbus
     * @param deviceId
     * @param canBus
     * @return a new CANcoderEx instance
     */
    public static CANcoderEx create(int deviceId, CANBus canBus) {
        return new CANcoderEx(deviceId, canBus);
    }
    
    /**
     * Creates a new CANcoderEx instance with the specified
     * device id and the default (rio) canbus.
     * @param deviceId
     * @return a new CANcoderEx instance
     */
    public static CANcoderEx create(int deviceId) {
        return new CANcoderEx(deviceId, DroidRageConstants.rioCanBus);
    }

    public static CANcoderEx createWithConstants(EncoderConstants constants) {
        return new CANcoderEx(constants);
    }

    public CANcoderEx withConfiguration(CANcoderConfiguration config) {
        encoder.getConfigurator().apply(config);
        return this;
    }

    public void resetPosition(Angle resetAngle) {
        encoder.setPosition(resetAngle);
    }
}