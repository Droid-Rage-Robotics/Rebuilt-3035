package frc.utility.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public boolean mainMotorConnected = false;
        public boolean encoderConnected = true;

        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;

        public double closedLoopReferenceMeters = 0.0;
        public double closedLoopReferenceVelocityMetersPerSec = 0.0;
        public double closedLoopErrorMeters = 0.0;
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setPosition(Distance position) {}

    default void setPositionMeters(double positionMeters) {
        setPosition(Meters.of(positionMeters));
    }

    default void setVoltage(Voltage voltage) {}

    default void resetEncoder(Distance position) {}

    default void stop() {
        setVoltage(Volts.zero());
    }
}
