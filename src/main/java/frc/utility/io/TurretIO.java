package frc.utility.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        public boolean motorConnected = false;
        public boolean encoderConnected = true;

        public Angle position = Rotations.zero();
        public AngularVelocity velocity = RotationsPerSecond.zero();
        public Voltage appliedVoltage = Volts.zero();
        public Current statorCurrent = Amps.zero();

        public Angle closedLoopReference = Rotations.zero();
        public AngularVelocity closedLoopReferenceVelocity = RotationsPerSecond.zero();
        public Angle closedLoopError = Rotations.zero();
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void setPosition(Angle angle) {}

    default void setVoltage(Voltage voltage) {}

    default void resetEncoder(Angle angle) {}

    default void stop() {
        setVoltage(Volts.zero());
    }
}