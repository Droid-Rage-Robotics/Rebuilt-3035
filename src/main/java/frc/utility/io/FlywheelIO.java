package frc.utility.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface FlywheelIO {
    @AutoLog
    class FlywheelIOInputs {
        public boolean mainMotorConnected = false;

        public Angle position = Rotations.zero();
        public AngularVelocity velocity = RotationsPerSecond.zero();
        public Voltage appliedVoltage = Volts.zero();
        public Current statorCurrent = Amps.zero();
        public Current torqueCurrent = Amps.zero();

        public AngularVelocity closedLoopReference = RotationsPerSecond.zero();
        public AngularVelocity closedLoopError = RotationsPerSecond.zero();
    }

    default void updateInputs(FlywheelIOInputs inputs) {}

    default void setVelocity(AngularVelocity velocity) {}

    default void setVoltage(Voltage voltage) {}

    default void stop() {
        setVoltage(Volts.zero());
    }
}