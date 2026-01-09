package frc.utility;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.DroidRageConstants;

public class CANrangeEx implements Sendable {
    private final CANrange range;
    private final CANrangeConfiguration config;
    private final int deviceId;

    private CANrangeEx(int deviceId, CANBus canBus) {
        this.range = new CANrange(deviceId, canBus);
        this.config = new CANrangeConfiguration();
        this.deviceId = deviceId;
    }

    public static CANrangeEx create(int deviceId) {
        return new CANrangeEx(deviceId, DroidRageConstants.rioCanBus);
    }

    public static CANrangeEx create(int deviceId, CANBus canBus) {
        return new CANrangeEx(deviceId, canBus);
    }

    public CANrangeEx withFOVCenterX(double value) {
        config.FovParams.FOVCenterY = value;
        configure(config);
        return this;
    }

    public CANrangeEx withFOVCenterY(double value) {
        config.FovParams.FOVCenterY = value;
        configure(config);
        return this;
    }

    public CANrangeEx withFOVRangeX(double value) {
        config.FovParams.FOVRangeX = value;
        configure(config);
        return this;
    }

    public CANrangeEx withFOVRangeY(double value) {
        config.FovParams.FOVRangeY = value;
        configure(config);
        return this;
    }

    public int getDeviceId() {
        return deviceId;
    }

    /**
     * Distance to the nearest object in the configured field of view of
     * the CANrange. Units are in meters.
     * 
     * @return distance as a double
     */
    public double getDistance() {
        return range.getDistance().getValueAsDouble();
    }

    /**
     * Whether the CANrange detects an object using the configured
     * proximity parameters.
     * 
     * @return isDetected boolean
     */
    public boolean getIsDetected() {
        return range.getIsDetected().getValue();
    }

    private void configure(CANrangeConfiguration config) {
        range.getConfigurator().apply(config);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Distance", this::getDistance, null);
        builder.addBooleanProperty("isDetected", this::getIsDetected, null);
    }
}
