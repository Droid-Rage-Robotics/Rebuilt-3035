package frc.utility.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public boolean motorConnected = false;
        public boolean encoderConnected = true;

        public Angle position = Radians.zero();
        public AngularVelocity velocity = RadiansPerSecond.zero();
        public Voltage appliedVoltage = Volts.zero();
        public Current statorCurrent = Amps.zero();
        public Temperature temp = Celsius.zero();

        public Angle closedLoopReference = Radians.zero();
        public AngularVelocity closedLoopReferenceVelocity = RadiansPerSecond.zero();
        public Angle closedLoopError = Radians.zero();
    }

    default void updateInputs(ArmIOInputs inputs) {}

    default void setGoalAngle(Angle angle) {}

    default void setVoltage(double volts) {
        setVoltage(Volts.of(volts));
    }

    default void setVoltage(Voltage volts) {}

    default void resetEncoder(Angle angle) {}

    default void stop() {
        setVoltage(Volts.zero());
    }
}