package frc.utility.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        public boolean motorConnected = false;
        public boolean encoderConnected = true;

        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;

        public double closedLoopReferenceRad = 0.0;
        public double closedLoopReferenceVelocityRadPerSec = 0.0;
        public double closedLoopErrorRad = 0.0;
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void setPosition(Angle angle) {}

    default void setPositionRad(double angleRad) {
        setPosition(Radians.of(angleRad));
    }

    default void setVoltage(Voltage voltage) {}

    default void resetEncoder(Angle angle) {}

    default void stop() {
        setVoltage(Volts.zero());
    }
}
