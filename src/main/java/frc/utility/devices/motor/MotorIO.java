package frc.utility.devices.motor;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface MotorIO {
    @AutoLog
    class MotorIOInputs {
        public boolean connected = false;

        public Angle position = Rotations.zero();
        public AngularVelocity velocity = RotationsPerSecond.zero();

        public Voltage appliedVolts = Volts.zero();
        public Current supplyCurrent = Amps.zero();
        public Current statorCurrent = Amps.zero();
        public Current torqueCurrent = Amps.zero();

        public Temperature temp = Celsius.zero();
    }

    default void updateInputs(MotorIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setVoltage(Voltage volts) {}

    default void setPower(double percent) {}

    default void setControl(ControlRequest request) {}

    default void stop() {}

    default void setPosition(double rotations) {}

    default void setPosition(Angle angle) {}

    default void setBrakeMode(boolean brake) {}

    default void setCurrentLimits(double supplyLimit, double statorLimit) {}
}