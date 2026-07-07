package frc.utility.io.devices;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface EncoderIO {
    @AutoLog
    class EncoderIOInputs {
        public boolean connected = false;

        public double absolutePositionRotations = 0.0;
        public double positionRotations = 0.0;
        public double velocityRotationsPerSecond = 0.0;
    }

    default void updateInputs(EncoderIOInputs inputs) {}

    default int getDeviceId() {
        return -1;
    }

    default void resetPosition(double positionRotations) {}

    default void resetPosition(Angle position) {
        resetPosition(position.in(Rotations));
    }
}
