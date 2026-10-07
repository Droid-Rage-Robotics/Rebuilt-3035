package frc.utility.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public int mainMotorIndex = 0;
        public boolean encoderConnected = true;

        // Array indices match the constructor motor order, including the leader.
        public int[] motorIds = new int[0];
        public boolean[] motorConnected = new boolean[0];
        public double[] motorAppliedVolts = new double[0];
        public double[] motorStatorCurrentAmps = new double[0];
        public double[] motorSupplyCurrentAmps = new double[0];
        public double[] motorTorqueCurrentAmps = new double[0];
        public double[] motorTempCelsius = new double[0];

        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;

        public double closedLoopReferenceMeters = 0.0;
        public double closedLoopReferenceVelocityMetersPerSec = 0.0;
        public double closedLoopErrorMeters = 0.0;
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setEnabled(boolean isEnabled) {}

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
