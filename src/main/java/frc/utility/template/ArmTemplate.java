package frc.utility.template;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.Dashboard;
import frc.utility.encoder.CANcoderEx;
import frc.utility.encoder.EncoderConstants;
import frc.utility.motor.MotorBase;
import frc.utility.motor.MotorConstants;
import frc.utility.motor.TalonEx;
import frc.utility.template.SubsystemConstants.EncoderType;

public class ArmTemplate extends SubsystemBase implements Dashboard {
    protected final MotorBase[] motors;
    protected final ProfiledPIDController controller;
    protected final ArmFeedforward feedforward;
    private final double minAngleRad;
    private final double maxAngleRad;
    private final double conversionFactor;
    private final double encoderOffsetRad;
    protected final int mainNum;
    protected final String name;
    private final Optional<CANcoderEx> encoder;

    private Rotation2d goalAngle = Rotation2d.fromRadians(0);

    public ArmTemplate(
        boolean isEnabled,
        ProfiledPIDController controller,
        ArmFeedforward feedforward,
        SubsystemConstants constants,
        EncoderConstants encoderConstants,
        MotorConstants... motorConstants
    ){
        this.controller=controller;
        this.feedforward=feedforward;
        this.minAngleRad=constants.lowerLimit;
        this.maxAngleRad=constants.upperLimit;
        this.conversionFactor=constants.conversionFactor;
        this.encoderOffsetRad=constants.offset;
        this.mainNum=constants.mainNum;
        this.name=constants.name;

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

        TelemetryUtils.registerDashboard(this);
    }

    /* ---------------- Dashboard ---------------- */

    @Override
    public void elasticInit() {
        SmartDashboard.putData(name, this);
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

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

        double ffOut = feedforward.calculate(setpoint.position, setpoint.velocity);

        setVoltage(pidOut + ffOut);
        //ks * Math.signum(velocity) + kg + kv * velocity + ka * acceleration; ^^
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    /* ---------------- Commands ---------------- */
    
    public Command setTargetPositionCommand(double degree){
        return new InstantCommand(()->setTargetPositionDegrees(degree));
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
    
    protected void setVoltage(double voltage) {
        double angle = getCurrentAngle().getRadians();
        if ((angle > maxAngleRad && voltage > 0) ||
            (angle < minAngleRad && voltage < 0)) {
            voltage = 0;
        }
        for (MotorBase motor: motors) {
            motor.setVoltage(voltage);
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

    public boolean atSetpoint(){
        return controller.atSetpoint();
    }
    
    public double getSetpointError() {
        return controller.getPositionError();
    }
}
