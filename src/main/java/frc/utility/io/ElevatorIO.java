package frc.utility.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public boolean mainMotorConnected = false;
        public boolean encoderConnected = true;

        public Distance position = Meters.zero();
        public LinearVelocity velocity = MetersPerSecond.zero();
        public Voltage appliedVoltage = Volts.zero();
        public Current statorCurrent = Amps.zero();

        public Distance closedLoopReference = Meters.zero();
        public LinearVelocity closedLoopReferenceVelocity = MetersPerSecond.zero();
        public Distance closedLoopError = Meters.zero();
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setPosition(Distance position) {}

    default void setVoltage(Voltage voltage) {}

    default void resetEncoder(Distance position) {}

    default void stop() {
        setVoltage(Volts.zero());
    }
}