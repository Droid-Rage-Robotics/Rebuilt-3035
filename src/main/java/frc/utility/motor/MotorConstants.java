package frc.utility.motor;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class MotorConstants {
    private final TalonFXConfiguration config = new TalonFXConfiguration();
    
    public int deviceId;
    public CANBus canBus;
    public double conversionFactor = 1;
    public Subsystem subsystem;
    public boolean isEnabled;
    
    /**
     * Determines the direction that the motor will
     * rotate when given a positive control request
     */
    public enum Direction {
        /**
         * Clockwise rotation is positive for the motor.
         */
        Forward,
        /**
         * Counterclockwise rotation is positive for the
         * motor.
         */
        Reversed,
    }

    /**
     * Determines motor behavior when not recieving
     * commands for voltage/power
     */
    public enum ZeroPowerMode {
        /**
         * Motor will resist movement when not recieving
         * commands for voltage/power.
         */
        Brake,

        /**
         * Motor will allow free movement when not recieving
         * commands for voltage/power.
         */
        Coast,
    }
    

    /**
     * Used to enable/disable the motor.
     * @param isEnabled
     * @return MotorConstants (for call chaining)
     */
    public MotorConstants withIsEnabled(boolean isEnabled) {
        this.isEnabled=isEnabled;
        return this;
    }

    /**
     * Used to set the canbus of the motor.
     * @param canBus the canbus the motor is on
     * @return MotorConstants (for call chaining)
     */
    public MotorConstants withCANBus(CANBus canBus) {
        this.canBus=canBus;
        return this;
    }

    /**
     * Used to set the deviceid of the motor
     * @param deviceid the deviceid of the motor
     * @return MotorConstants (for call chaining)
     */
    public MotorConstants withDeviceId(int deviceId) {
        this.deviceId=deviceId;
        return this;
    }

    /**
     * Used to set a conversion factor to be applied to
     * the raw position/velocity of the motor. Defaults
     * to 1.
     * @param conversionFactor
     * @return MotorConstants (for call chaining)
     */
    public MotorConstants withConversionFactor(double conversionFactor) {
        this.conversionFactor=conversionFactor;
        return this;
    }

    /**
     * Used to set the subsystem that this motor is a
     * part of.
     * @param subsystem
     * @return MotorConstants (for call chaining)
     */
    public MotorConstants withSubsystem(Subsystem subsystem){
        this.subsystem = subsystem;
        return this;
    }

    /**
     * Used to set the supply current limit of the motor.
     * Defaults to 70 Amps.
     * @param value
     * @return MotorConstants (for call chaining)
     */
    public MotorConstants withSupplyCurrentLimit(double value) {
        config.CurrentLimits.SupplyCurrentLimit = value;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        return this;
    }

    /**
     * Used to set the stator current limit of the motor.
     * Defaults to 120 Amps.
     * @param value
     * @return MotorConstants (for call chaining)
     */
    public MotorConstants withStatorCurrentLimit(double value) {
        config.CurrentLimits.StatorCurrentLimit = value;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        return this;
    }

    /**
     * Used to set the direction of the motor. Defaults to
     * forward (clockwise-positive motion). Can be reversed
     * to create counter-clockwise positive motion.
     * @param direction
     * @return MotorConstants (for call chaining)
     */
    public MotorConstants withDirection(Direction direction) {
        config.MotorOutput.Inverted = switch (direction) {
            case Forward -> InvertedValue.Clockwise_Positive;
            case Reversed -> InvertedValue.CounterClockwise_Positive;
        };
        return this;
    }

    /**
     * Used to set the motor behavior when given zero output
     * voltage.
     * @param mode
     * @return MotorConstants (for call chaining)
     */
    public MotorConstants withIdleMode(NeutralModeValue mode) {
        config.MotorOutput.NeutralMode=mode;
        return this;
    }

    public TalonFXConfiguration getConfig() {
        return this.config;
    }
}
