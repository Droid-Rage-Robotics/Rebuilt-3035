package frc.utility;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Uses a PID Controller to quickly turn the robot to a specified angle. Once the robot is within
 * a certain tolerance of the goal angle, a second PID controller is used to hold the robot at that
 * angle.
 */
public class RotationController implements Sendable {
    private final PIDController controller;
    private final PIDController holdController;

    private double calculation = 0;
    private double holdCalculation = 0;

    private final double feedbackSetpoint = 0.35;
    private final double tolerance = Math.toRadians(2);
    
    private RotationController() {
        controller = new PIDController(0, 0, 0);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(tolerance * 2);
        // constraints = new Constraints(config.maxAngularVelocity, config.maxAngularAcceleration);

        holdController = new PIDController(0, 0,0);
        holdController.enableContinuousInput(-Math.PI, Math.PI);
        holdController.setTolerance(tolerance);
    }

    public static RotationController create() {
        return new RotationController();
    }

    public RotationController withApproachPID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
        return this;
    }

    public RotationController withHoldPID(double kP, double kI, double kD) {
        holdController.setPID(kP, kI, kD);
        return this;
    }

    public double calculate(double measurement, double goalRadians) {
        calculation = controller.calculate(measurement, goalRadians);
        holdCalculation = holdController.calculate(measurement, goalRadians);

        return atSetpoint() ? calculation : holdCalculation;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Approach Output", () -> calculation, null);
        builder.addDoubleProperty("Hold Output", () -> holdCalculation, null);
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    public boolean atHoldSetpoint() {
        return holdController.atSetpoint();
    }

    public boolean atFeedbackSetpoint() {
        return Math.abs(calculation) <= feedbackSetpoint;
    }
    
    public void reset() {
        controller.reset();
        holdController.reset();
    }
}