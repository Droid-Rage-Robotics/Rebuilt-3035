package frc.utility;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SimpleMotorFeedforwardConstants {
    /** S */
    public final double kS;
    /** V */
    public final double kV;
    /** A */
    public final double kA;

    /**
     * Create a new SimpleMotorFeedforwardConstants object
     *
     * @param kP P
     * @param kI I
     * @param kD D
     */
    public SimpleMotorFeedforwardConstants(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Create a new SimpleMotorFeedforwardConstants object
     *
     * @param kS S
     * @param kV V
     */
    public SimpleMotorFeedforwardConstants(double kS, double kV) {
        this(kS, kV, 0);
    }

    /**
     * Create a new SimpleMotorFeedforwardConstants object
     *
     * @param kS S
     */
    public SimpleMotorFeedforwardConstants(double kS) {
        this(kS, 0, 0);
    }

    public SimpleMotorFeedforward toSimpleMotorFeedforward() {
        return new SimpleMotorFeedforward(kS, kV, kA);
    }
}
