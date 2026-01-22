package frc.utility.encoder;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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

    /**
     * Modifies this configuration's AbsoluteSensorDiscontinuityPoint parameter and returns itself for
     * method-chaining and easier to use config API.
     * <p>
     * The positive discontinuity point of the absolute sensor in
     * rotations. This determines the point at which the absolute sensor
     * wraps around, keeping the absolute position (after offset) in the
     * range [x-1, x).
     * 
     * <ul>
     *   <li> Setting this to 1 makes the absolute position unsigned [0,
     * 1)
     *   <li> Setting this to 0.5 makes the absolute position signed
     * [-0.5, 0.5)
     *   <li> Setting this to 0 makes the absolute position always
     * negative [-1, 0)
     * </ul>
     * 
     * Many rotational mechanisms such as arms have a region of motion
     * that is unreachable. This should be set to the center of that
     * region of motion, in non-negative rotations. This affects the
     * position of the device at bootup.
     * <p>
     * For example, consider an arm which can travel from -0.2 to 0.6
     * rotations with a little leeway, where 0 is horizontally forward.
     * Since -0.2 rotations has the same absolute position as 0.8
     * rotations, we can say that the arm typically does not travel in the
     * range (0.6, 0.8) rotations. As a result, the discontinuity point
     * would be the center of that range, which is 0.7 rotations. This
     * results in an absolute sensor range of [-0.3, 0.7) rotations.
     * <p>
     * Given a total range of motion less than 1 rotation, users can
     * calculate the discontinuity point using mean(lowerLimit,
     * upperLimit) + 0.5. If that results in a value outside the range [0,
     * 1], either cap the value to [0, 1], or add/subtract 1.0 rotation
     * from your lower and upper limits of motion.
     * <p>
     * On a Talon motor controller, this is only supported when using the
     * PulseWidth sensor source.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 1.0
     *   <li> <b>Default Value:</b> 0.5
     *   <li> <b>Units:</b> rotations
     * </ul>
     *
     * @param newAbsoluteSensorDiscontinuityPoint Parameter to modify
     * @return Itself
     */
    public CANcoderEx withAbsoluteSensorDiscontinuityPoint(double value) {
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint=value;
        withConfiguration(config);
        return this;
    }

    /**
     * Modifies this configuration's MagnetOffset parameter and returns itself for
     * method-chaining and easier to use config API.
     * <p>
     * This offset is added to the reported position, allowing the
     * application to trim the zero position.  When set to the default
     * value of zero, position reports zero when magnet north pole aligns
     * with the LED.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -1
     *   <li> <b>Maximum Value:</b> 1
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations
     * </ul>
     *
     * @param newMagnetOffset Parameter to modify
     * @return Itself
     */
    public CANcoderEx withMagnetOffset(double value) {
        config.MagnetSensor.MagnetOffset=value;
        withConfiguration(config);
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
     * @return Itself
     */
    public CANcoderEx withDirection(SensorDirectionValue value) {
        config.MagnetSensor.SensorDirection=value;
        withConfiguration(config);
        return this;
    }

    public CANcoderEx withConfiguration(CANcoderConfiguration config) {
        encoder.getConfigurator().apply(config);
        return this;
    }

    public double getAbsolutePosition() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getVelocity() {
        return encoder.getVelocity().getValueAsDouble();
    }

    public int getDeviceId() {
        return deviceId;
    }
}