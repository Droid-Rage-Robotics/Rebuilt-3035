package frc.utility.devices.motor;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class MotorIOTalonFX implements MotorIO {
    private final TalonFX motor;
    private final TalonFXConfiguration config;

    private final double conversionFactor;
    private boolean enabled;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temp;

    private final Debouncer connectedDebounce = new Debouncer(0.5);

    public MotorIOTalonFX(MotorConstants constants) {
        this(constants, 1.0);
    }

    public MotorIOTalonFX(MotorConstants constants, double conversionFactor) {
        this.motor = new TalonFX(constants.deviceId, constants.canBus);
        this.config = constants.getConfig();
        this.conversionFactor = conversionFactor;
        this.enabled = constants.isEnabled;

        config.MotorOutput.Inverted = switch (constants.direction) {
            case Forward -> com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
            case Reversed -> com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        };

        motor.getConfigurator().apply(config, 0.25);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        appliedVolts = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        statorCurrent = motor.getStatorCurrent();
        torqueCurrent = motor.getTorqueCurrent();
        temp = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            position,
            velocity,
            appliedVolts
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            10.0,
            supplyCurrent,
            statorCurrent,
            torqueCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            2.0,
            temp
        );

        ParentDevice.optimizeBusUtilizationForAll(motor);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        StatusCode fastStatus = BaseStatusSignal.refreshAll(
            position,
            velocity,
            appliedVolts
        );

        BaseStatusSignal.refreshAll(
            supplyCurrent,
            statorCurrent,
            torqueCurrent,
            temp
        );

        inputs.connected = connectedDebounce.calculate(fastStatus.isOK());

        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();

        inputs.appliedVolts = appliedVolts.getValue();
        inputs.supplyCurrent = supplyCurrent.getValue();
        inputs.statorCurrent = statorCurrent.getValue();
        inputs.torqueCurrent = torqueCurrent.getValue();
        inputs.temp = temp.getValue();
    }

    @Override
    public void setVoltage(double volts) {
        if (enabled) {
            motor.setVoltage(volts);
        }
    }

    @Override
    public void setVoltage(Voltage volts) {
        setVoltage(volts.in(Volts));
    }

    @Override
    public void setPower(double percent) {
        if (enabled) {
            motor.set(percent);
        }
    }

    @Override
    public void setControl(ControlRequest request) {
        if (enabled) {
            motor.setControl(request);
        }
    }

    @Override
    public void stop() {
        motor.setVoltage(0.0);
    }

    @Override
    public void setPosition(double rotations) {
        motor.setPosition(rotations);
    }

    @Override
    public void setPosition(Angle angle) {
        motor.setPosition(angle);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        config.MotorOutput.NeutralMode =
            brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        motor.getConfigurator().apply(config, 0.25);
    }

    @Override
    public void setCurrentLimits(double supplyLimit, double statorLimit) {
        config.CurrentLimits.SupplyCurrentLimit = supplyLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = statorLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(config, 0.25);
    }

    public void applyConfig() {
        motor.getConfigurator().apply(config, 0.25);
    }

    public TalonFXConfiguration getConfig() {
        return config;
    }

    public TalonFX getMotor() {
        return motor;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
}