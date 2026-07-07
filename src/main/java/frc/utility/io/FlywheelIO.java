package frc.utility.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface FlywheelIO {
    @AutoLog
    class FlywheelIOInputs {
        public boolean mainMotorConnected = false;

        public double positionRotations = 0.0;
        public double velocityRotationsPerSecond = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;

        public double closedLoopReferenceRotationsPerSecond = 0.0;
        public double closedLoopErrorRotationsPerSecond = 0.0;
    }

    default void updateInputs(FlywheelIOInputs inputs) {}

    default void setVelocity(AngularVelocity velocity) {}

    default void setVelocityRotationsPerSecond(double velocityRotationsPerSecond) {
        setVelocity(RotationsPerSecond.of(velocityRotationsPerSecond));
    }

    default void setVoltage(Voltage voltage) {}

    default void stop() {
        setVoltage(Volts.zero());
    }
}
