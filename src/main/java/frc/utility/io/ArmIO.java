package frc.utility.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public boolean motorConnected = false;
        public boolean encoderConnected = true;

        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempCelsius = 0.0;

        public double closedLoopReferenceRad = 0.0;
        public double closedLoopReferenceVelocityRadPerSec = 0.0;
        public double closedLoopErrorRad = 0.0;
    }

    default void updateInputs(ArmIOInputs inputs) {}

    default void setGoalAngle(Angle angle) {}

    default void setGoalAngleRad(double angleRad) {
        setGoalAngle(Radians.of(angleRad));
    }

    default void setVoltage(double volts) {
        setVoltage(Volts.of(volts));
    }

    default void setVoltage(Voltage volts) {}

    default void resetEncoder(Angle angle) {}

    default void stop() {
        setVoltage(Volts.zero());
    }
}
