package frc.utility.template;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utility.LimelightEx;
import frc.utility.DashboardUtils.Dashboard;
import frc.utility.motor.MotorBase;

public class TurretTemplate extends SubsystemBase implements Dashboard {
    private final MotorBase[] motors;
    private final int mainNum;
    private final LimelightEx limelight;

    private final ProfiledPIDController controller;
    private final SimpleMotorFeedforward feedforward;

    private final double minAngleRad;
    private final double maxAngleRad;
    private final double conversionFactor;
    private final double encoderOffsetRad;

    private final boolean isEnabled;

    private Rotation2d goalAngle = Rotation2d.fromRadians(0);

    public TurretTemplate(
        MotorBase[] motors,
        int mainNum,
        LimelightEx limelight,
        ProfiledPIDController controller,
        SimpleMotorFeedforward feedforward,
        double minAngleRad,
        double maxAngleRad,
        double conversionFactor,
        double encoderOffsetRad,
        boolean isEnabled
    ) {
        this.motors=motors;
        this.mainNum=mainNum;
        this.limelight=limelight;
        this.controller=controller;
        this.feedforward=feedforward;
        this.minAngleRad=minAngleRad;
        this.maxAngleRad=maxAngleRad;
        this.conversionFactor=conversionFactor;
        this.encoderOffsetRad=encoderOffsetRad;
        this.isEnabled=isEnabled;

        controller.enableContinuousInput(-Math.PI, Math.PI);

        for (MotorBase motor : motors) {
            motor.withIsEnabled(isEnabled);
        }
    }

    /* ---------------- Dashboard ---------------- */

    @Override
    public void elasticInit() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/Reset Encoder", runOnce(this::resetEncoder));
    }

    @Override public void practiceWriters() {}
    @Override public void alerts() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Goal Angle (deg)", () -> getGoalAngle().getDegrees(), null);
        builder.addDoubleProperty("Current Angle (deg)", () -> getCurrentAngle().getDegrees(), null);
        builder.addDoubleProperty("Position Setpoint (deg)", this::getPositionSetpoint, null);
        builder.addDoubleProperty("Velocity Setpoint (deg/s)", this::getVelocitySetpoint, null);
        builder.addDoubleProperty("Current Velocity (deg/s)", () -> Math.toDegrees(getVelocity()), null);
        builder.addDoubleProperty("Applied Voltage", this::getVoltage, null);
        builder.addDoubleProperty("Position Error (deg)", () -> Math.toDegrees(controller.getPositionError()), null);
    }

    /* ---------------- Periodic Control Loop ---------------- */

    @Override
    public void periodic() {
        double currentAngleRad = getCurrentAngle().getRadians();

        double pidOut = controller.calculate(currentAngleRad);
        var setpoint = controller.getSetpoint();

        double ffOut = feedforward.calculate(setpoint.velocity);

        setVoltage(pidOut + ffOut);
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    /* ---------------- Commands ---------------- */

    public Command setTargetPositionCommand(double degrees) {
        return new InstantCommand(() -> setTargetPositionDegrees(degrees), this);
    }

    /* ---------------- Manual Goal Control ---------------- */

    public void setTargetPositionDegrees(double degrees) {
        setGoalAngle(Rotation2d.fromDegrees(degrees));
    }

    public void setGoalAngle(Rotation2d angle) {
        double clamped = MathUtil.clamp(
            angle.getRadians(),
            minAngleRad,
            maxAngleRad
        );

        goalAngle = new Rotation2d(clamped);
        controller.setGoal(clamped);
    }

    public Rotation2d getGoalAngle() {
        return goalAngle;
    }

    public double getVelocitySetpoint() {
        return Math.toDegrees(controller.getSetpoint().velocity);
    }

    public double getPositionSetpoint() {
        return Math.toDegrees(controller.getSetpoint().position);
    }

    /* ---------------- Vision Aiming ---------------- */

    public void aimWithLimelight() {
        if (!limelight.getTV()) return;

        double txDeg = limelight.getTX();
        Rotation2d currentAngle = getCurrentAngle();

        // Shift the goal by the Limelight error
        Rotation2d newGoal = currentAngle.minus(Rotation2d.fromDegrees(txDeg));
        setGoalAngle(newGoal);
    }
    
    /* ---------------- Sensor Access ---------------- */

    public Rotation2d getCurrentAngle() {
        double motorPos = motors[mainNum].getPosition(); // motor units
        double angleRad = motorPos * conversionFactor + encoderOffsetRad;
        return new Rotation2d(angleRad);
    }

    public double getVelocity() {
        return motors[mainNum].getVelocity() * conversionFactor;
    }
    
    public double getVoltage() {
        return motors[mainNum].getVoltage();
    }

    /* ---------------- Motor Control ---------------- */

    public void setVoltage(double voltage) {
        if (isEnabled) {
            for (MotorBase motor: motors) {
            motor.setVoltage(voltage);
        }
        }
    }
    
    public void resetEncoder() {
        for (MotorBase motor: motors) {
            motor.resetEncoder(0);
        }
    }

    /* ---------------- Utility ---------------- */
    
    public MotorBase getMotor(){
        return motors[mainNum];
    }
    
    public MotorBase[] getAllMotor() {
        return motors;
    }

    public boolean atGoal(){
        return controller.atGoal();
    }
}
