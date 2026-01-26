package frc.utility.template;

import java.util.Optional;

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
import frc.utility.encoder.CANcoderEx;
import frc.utility.encoder.EncoderConstants;
import frc.utility.motor.MotorBase;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.TalonEx;
import frc.utility.template.SubsystemConstants.EncoderType;

public class TurretTemplate extends SubsystemBase implements Dashboard {
    private final MotorBase[] motors;
    private final Optional<CANcoderEx> encoder;
    private final LimelightEx limelight;

    private final ProfiledPIDController controller;
    private final SimpleMotorFeedforward feedforward;

    private final double minAngleRad;
    private final double maxAngleRad;
    private final double conversionFactor;
    private final int mainNum;
    private final double encoderOffsetRad;

    private final boolean isEnabled;

    private Rotation2d goalAngle = Rotation2d.fromRadians(0);

    public TurretTemplate(
        boolean isEnabled,
        LimelightEx limelight,
        ProfiledPIDController controller,
        SimpleMotorFeedforward feedforward,
        SubsystemConstants constants,
        EncoderConstants encoderConstants,
        MotorConstants... motorConstants
    ) {
        this.mainNum=constants.mainNum;
        this.limelight=limelight;
        this.controller=controller;
        this.feedforward=feedforward;
        this.minAngleRad=constants.lowerLimit;
        this.maxAngleRad=constants.upperLimit;
        this.conversionFactor=constants.conversionFactor;
        this.encoderOffsetRad=constants.offset;
        this.isEnabled=isEnabled;

        controller.enableContinuousInput(-Math.PI, Math.PI);

        if (constants.encoderType == EncoderType.ABSOLUTE) {
            if (encoderConstants == null) {
                throw new NullPointerException("Encoder constants required for absolute encoder");
            }
            this.encoder = Optional.of(CANcoderEx.createWithConstants(encoderConstants));
        } else {
            this.encoder = Optional.empty();
        }
        
        
        this.motors = new MotorBase[motorConstants.length];
        
        for (MotorConstants m_motorConstants : motorConstants) {
            m_motorConstants.subsystem=this;
            m_motorConstants.isEnabled=isEnabled;
        }

        for (int i = 0; i < motorConstants.length; i++) {
            this.motors[i] = TalonEx.createWithConstants(motorConstants[i]);
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
    
    /* ---------------- Sensor Access ---------------- */

    public Rotation2d getCurrentAngle() {
        var raw = encoder
            .map(enc -> enc.getAbsolutePosition())
            .orElse(motors[mainNum].getPosition());

        double angleRad = raw * conversionFactor + encoderOffsetRad;
        return new Rotation2d(angleRad);
    }

    public double getVelocity() {
        return encoder
            .map(enc -> enc.getVelocity() * conversionFactor)
            .orElse(motors[mainNum].getVelocity() * conversionFactor);
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
