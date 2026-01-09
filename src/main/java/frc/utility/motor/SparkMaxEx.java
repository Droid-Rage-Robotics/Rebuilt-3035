package frc.utility.motor;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SparkMaxEx extends MotorBase {
    private final SparkMax motor;
    private final SparkMaxConfig config;
    private final int deviceId;
    private double conversionFactor = 1;
    private boolean isEnabled;
    private Subsystem subsystem;

    public SparkMaxEx(int deviceId, MotorType motorType) {
        this.motor = new SparkMax(deviceId, motorType);
        this.config = new SparkMaxConfig();
        this.deviceId = deviceId;
    }

    /**
     * Creates a new SparkMaxEx instance with the specified
     * device id and motor type (brushed/brushless)
     * @param deviceId
     * @param motorType
     * @return a new SparkMaxEx instance
     */
    public static SparkMaxEx create(int deviceId, MotorType motorType) {
        return new SparkMaxEx(deviceId, motorType);
    }

    /**
     * Creates a new SparkMaxEx instance with the specified
     * device id and a default motor type of brushless
     * @param deviceId
     * @return a new SparkMaxEx instance
     */
    public static SparkMaxEx create(int deviceId) {
        return new SparkMaxEx(deviceId, MotorType.kBrushless);
    }

    /**
     * Used to enable/disable the motor.
     * @param isEnabled
     * @return SparkMaxEx (for call chaining)
     */
    @Override
    public SparkMaxEx withIsEnabled(boolean isEnabled) {
        this.isEnabled=isEnabled;
        return this;
    }

    /**
     * Used to set a conversion factor to be applied to
     * the raw position/velocity of the motor. Defaults
     * to 1.
     * @param conversionFactor
     * @return SparkMaxEx (for call chaining)
     */
    public SparkMaxEx withConversionFactor(double conversionFactor) {
        this.conversionFactor=conversionFactor;
        return this;
    }
    
    /**
     * Used to set the subsystem that this motor is a
     * part of.
     * @param subsystem
     * @return SparkMaxEx (for call chaining)
     */
    public SparkMaxEx withSubsystem(Subsystem subsystem) {
        this.subsystem=subsystem;
        return this;
    }

    /**
     * Used to set the supply current limit of the motor.
     * @param value
     * @return SparkMaxEx (for call chaining)
     */
    public SparkMaxEx withSupplyCurrentLimit(int value) {
        config.smartCurrentLimit(value);
        burnFlash();
        return this;
    }

    /**
     * Used to set the direction of the motor. Defaults to
     * forward (clockwise-positive motion). Can be reversed
     * to create counter-clockwise positive motion.
     * @param direction
     * @return SparkMaxEx (for call chaining)
     */
    public SparkMaxEx withDirection(Direction direction) {
        config.inverted(switch (direction) {
            case Forward -> false;
            case Reversed -> true;
        });
        burnFlash();
        return this;
    }

    /**
     * Used to set the motor behavior when given zero output
     * voltage.
     * @param mode
     * @return SparkMaxEx (for call chaining)
     */
    public SparkMaxEx withIdleMode(ZeroPowerMode mode) {
        config.idleMode(switch (mode) {
            case Brake -> IdleMode.kBrake;
            case Coast -> IdleMode.kCoast;
        });
        burnFlash();
        return this;
    }

    /**
     * Used to get the velocity of the motor. Default units
     * with default conversion factor of 1 are in rotations
     * per second. Custom conversion factors are automatically
     * applied.
     * @return the velocity of the motor
     */
    @Override
    public double getVelocity() {
        return (motor.getEncoder().getVelocity()/60) * conversionFactor;
    }

    /**
     * Used to get the position of the motor. Default units
     * with default conversion factor of 1 are in rotations.
     * Custom conversion factors are automatically applied.
     * @return the positon of the motor
     */
    @Override
    public double getPosition() {
        return motor.getEncoder().getPosition() * conversionFactor;
    }

    /**
     * Used to get the device id of the motor.
     * @return the device id as an int
     */
    @Override
    public int getDeviceId() {
        return deviceId;
    }

    /**
     * Used to get the temperature of the motor.
     * Default units are in Celsius.
     * @return the temperature as a double
     */
    @Override
    public double getTemp() {
        return motor.getMotorTemperature();
    }

    /**
     * Used to get the subsystem that the motor
     * is a part of.
     * @return a subsystem
     */
    @Override
    public Subsystem getSubsystem() {
        return subsystem;
    }

    /**
     * Used to get the applied voltage to the motor.
     * @return the voltage as a double
     */
    @Override
    public double getVoltage() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    /**
     * Used to get a connected absolute encoder.
     * @return an AbsoluteEncoder object
     */
    public AbsoluteEncoder getAbsoluteEncoder() {
        return motor.getAbsoluteEncoder();
    }

    /**
     * Used to get the raw SparkMax object. Make
     * sure you know what you are doing!
     * @return a SparkMax object
     */
    public SparkMax getMotor() {
        return motor;
    }

    /**
     * Used to reset the encoder of the motor to a
     * specific position in rotations.
     * @param value position in rotations
     */
    @Override
    public void resetEncoder(double value) {
        motor.getEncoder().setPosition(value);
    }
    
    /**
     * Used to apply a voltage to the motor. Does
     * nothing if the motor is disabled.
     * @param voltage voltage as a double
     */
    @Override
    public void setVoltage(double voltage) {
        if (isEnabled) {
            motor.setVoltage(voltage);
        }
    }

    /**
     * Used to apply a voltage to the motor. Does
     * nothing if the motor is disabled.
     * @param voltage voltage
     */
    @Override
    public void setVoltage(Voltage voltage) {
        if (isEnabled) {
            motor.setVoltage(voltage);
        }
    }

    /**
     * Used to control the motor in terms of power
     * as opposed to manually setting the voltage.
     * @param power speed in the range of -1 to 1
     */
    @Override
    public void setPower(double power) {
        if (isEnabled) {
            motor.set(power);
        }
    }

    /**
     * Disables the motor and sets the voltage to 0.
     */
    @Override
    public void stop() {
        withIsEnabled(false);
        motor.setVoltage(0);
    }
    
    /**
     * Applies the configs of the motor.
     */
    public void burnFlash() {
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setAbsoluteEncoderConfig(AbsoluteEncoderConfig config) {
        config.setSparkMaxDataPortConfig();
        this.config.absoluteEncoder.apply(config);
        burnFlash();
    }

    /**
     * Causes this controller's output to mirror the provided leader.
     *
     * <p>Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
     *
     * @param leader The motor controller to follow.
     * @param invert Set the follower to output opposite of the leader
     */
    public void follow(SparkMaxEx leader, boolean invert) {
        config.follow(leader.getMotor(), invert);
        burnFlash();
    }
}