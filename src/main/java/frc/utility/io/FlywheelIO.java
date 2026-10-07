package frc.utility.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

public interface FlywheelIO {
    @AutoLog
    class FlywheelIOInputs {
        public int mainMotorIndex = 0;

        // Array indices match the constructor motor order, including the leader.
        public int[] motorIds = new int[0];
        public boolean[] motorConnected = new boolean[0];
        public double[] motorAppliedVolts = new double[0];
        public double[] motorStatorCurrentAmps = new double[0];
        public double[] motorSupplyCurrentAmps = new double[0];
        public double[] motorTorqueCurrentAmps = new double[0];
        public double[] motorTempCelsius = new double[0];

        public double positionRotations = 0.0;
        public double velocityRotationsPerSecond = 0.0;

        public double closedLoopReferenceRotationsPerSecond = 0.0;
        public double closedLoopErrorRotationsPerSecond = 0.0;
    }

    default void updateInputs(FlywheelIOInputs inputs) {}

    default void setEnabled(boolean isEnabled) {}

    default void setVelocity(AngularVelocity velocity) {}

    default void setVelocityRotationsPerSecond(double velocityRotationsPerSecond) {
        setVelocity(RotationsPerSecond.of(velocityRotationsPerSecond));
    }

    default void setVoltage(Voltage voltage) {}

    default void stop() {
        setVoltage(Volts.zero());
    }
}
