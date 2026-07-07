package frc.utility.io.devices;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public interface MotorIO {
    @AutoLog
    class MotorIOInputs {
        public boolean connected = false;

        public double positionRotations = 0.0;
        public double velocityRotationsPerSecond = 0.0;

        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;

        public double tempCelsius = 0.0;
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
