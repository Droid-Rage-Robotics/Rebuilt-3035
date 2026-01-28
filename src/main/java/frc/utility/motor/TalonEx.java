package frc.utility.motor;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.DroidRageConstants;
import frc.utility.motor.MotorConstants.Direction;

public class TalonEx extends MotorBase {
    private final TalonFX motor;
    private final TalonFXConfiguration config;
    private final TalonFXConfigurator configurator;
    private final int deviceId;
    private final CANBus canBus;
    private double conversionFactor = 1;
    private Subsystem subsystem;
    private boolean isEnabled;
    
    private TalonEx(int deviceId, CANBus canBus) {
        this.motor = new TalonFX(deviceId, canBus);
        this.canBus = canBus;
        this.deviceId=deviceId;
        this.config = new TalonFXConfiguration(); // Use to change configs
        this.configurator = motor.getConfigurator(); // Use to apply configs
    }

    private TalonEx(MotorConstants constants) {
        this.motor = new TalonFX(constants.deviceId, constants.canBus);
        this.canBus = constants.canBus;
        this.deviceId = constants.deviceId;
        this.isEnabled = constants.isEnabled;
        this.config = constants.getConfig();
        this.configurator = motor.getConfigurator();
        configurator.apply(config);
    }

    /**
     * Creates a new TalonEx instance with the specified
     * device id and canbus
     * @param deviceId
     * @param canBus
     * @return a new TalonEx instance
     */
    public static TalonEx create(int deviceId, CANBus canBus) {
        return new TalonEx(deviceId, canBus);
    }

    /**
     * Creates a new TalonEx instance with the specified
     * device id and the default (rio) canbus.
     * @param deviceId
     * @return a new TalonEx instance
     */
    public static TalonEx create(int deviceId) {
        return new TalonEx(deviceId, DroidRageConstants.rioCanBus);
    }

    public static TalonEx createWithConstants(MotorConstants constants) {
        return new TalonEx(constants);
    }

    /**
     * Used to enable/disable the motor.
     * @param isEnabled
     * @return TalonEx (for call chaining)
     */
    @Override
    public TalonEx withIsEnabled(boolean isEnabled) {
        this.isEnabled=isEnabled;
        return this;
    }

    /**
     * Used to set a conversion factor to be applied to
     * the raw position/velocity of the motor. Defaults
     * to 1.
     * @param conversionFactor
     * @return TalonEx (for call chaining)
     */
    public TalonEx withConversionFactor(double conversionFactor) {
        this.conversionFactor=conversionFactor;
        return this;
    }

    /**
     * Used to set the subsystem that this motor is a
     * part of.
     * @param subsystem
     * @return TalonEx (for call chaining)
     */
    public TalonEx withSubsystem(Subsystem subsystem){
        this.subsystem = subsystem;
        return this;
    }

    /**
     * Used to set the supply current limit of the motor.
     * Defaults to 70 Amps.
     * @param value
     * @return TalonEx (for call chaining)
     */
    public TalonEx withSupplyCurrentLimit(double value) {
        config.CurrentLimits.SupplyCurrentLimit = value;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        configurator.apply(config);
        return this;
    }

    /**
     * Used to set the stator current limit of the motor.
     * Defaults to 120 Amps.
     * @param value
     * @return TalonEx (for call chaining)
     */
    public TalonEx withStatorCurrentLimit(double value) {
        config.CurrentLimits.StatorCurrentLimit = value;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        configurator.apply(config);
        return this;
    }

    /**
     * Used to set the direction of the motor. Defaults to
     * forward (clockwise-positive motion). Can be reversed
     * to create counter-clockwise positive motion.
     * @param direction
     * @return TalonEx (for call chaining)
     */
    public TalonEx withDirection(Direction direction) {
        config.MotorOutput.Inverted = switch (direction) {
            case Forward -> InvertedValue.Clockwise_Positive;
            case Reversed -> InvertedValue.CounterClockwise_Positive;
        };
        configurator.apply(config);
        return this;
    }

    /**
     * Used to set the motor behavior when given zero output
     * voltage.
     * @param mode
     * @return TalonEx (for call chaining)
     */
    public TalonEx withIdleMode(NeutralModeValue mode) {
        config.MotorOutput.NeutralMode=mode;
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
        return motor.getVelocity().getValueAsDouble() * conversionFactor;
    }

    @Override
    public TalonFXSimState getSimState() {
        return motor.getSimState();
    }

    /**
     * Used to get the position of the motor. Default units
     * with default conversion factor of 1 are in rotations.
     * Custom conversion factors are automatically applied.
     * @return the positon of the motor
     */
    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble() * conversionFactor;
    }

    /**
     * Used to get the temperature of the motor.
     * Default units are in Celsius.
     * @return the temperature as a double
     */
    @Override
    public double getTemp() {
        return motor.getDeviceTemp().getValueAsDouble();
    }

    /**
     * Used to get the applied voltage to the motor.
     * @return the applied voltage to the motor
     */
    @Override
    public double getVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Used to get the canbus of the motor.
     * @return a CANBus object
     */
    public CANBus getCANBus() {
        return canBus;
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
     * Used to get the subsystem that the motor
     * is a part of.
     * @return a subsystem
     */
    public Subsystem getSubsystem() {
        return this.subsystem;
    }

    /**
     * Used to get the raw TalonFX object. Make
     * sure you know what you are doing!
     * @return a TalonFX object
     */
    public TalonFX getMotor() {
        return motor;
    }

    /**
     * Used to reset the encoder of the motor to a
     * specific position in rotations.
     * @param value
     */
    @Override
    public void resetEncoder(double value) {
        motor.setPosition(value);
    }

    /**
     * Used to apply a voltage to the motor. Does
     * nothing if the motor is disabled.
     * @param voltage voltage
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
            motor.setVoltage(voltage.in(Volts));
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
}