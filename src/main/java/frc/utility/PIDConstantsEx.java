package frc.utility;

import edu.wpi.first.math.controller.PIDController;

/** PID constants used to create PID controllers */
public class PIDConstantsEx {
    /** P */
    public final double kP;
    /** I */
    public final double kI;
    /** D */
    public final double kD;

    /**
     * Create a new PIDConstants object
     *
     * @param kP P
     * @param kI I
     * @param kD D
     */
    public PIDConstantsEx(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Create a new PIDConstants object
     *
     * @param kP P
     * @param kD D
     */
    public PIDConstantsEx(double kP, double kD) {
        this(kP, 0, kD);
    }

    /**
     * Create a new PIDConstants object
     *
     * @param kP P
     */
    public PIDConstantsEx(double kP) {
        this(kP, 0, 0);
    }

    public PIDController toPIDController() {
        return new PIDController(kP, kI, kD);
    }
}
