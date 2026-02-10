package frc.utility.template;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.Dashboard;
import frc.utility.encoder.CANcoderEx;
import frc.utility.encoder.EncoderConstants;
import frc.utility.motor.TalonEx;
import frc.utility.motor.MotorConstants;
import frc.utility.template.SubsystemConstants.EncoderType;

public class TurretTemplate extends SubsystemBase implements Dashboard {
    private final TalonEx[] motors;
    private final Optional<CANcoderEx> encoder;
    private final ProfiledPIDController controller;
    private final SimpleMotorFeedforward feedforward;

    private final double minAngleRad;
    private final double maxAngleRad;
    private final double conversionFactor;
    private final int mainNum;
    private final SubsystemConstants constants;


    private final boolean isEnabled;

    private Rotation2d goalAngle = Rotation2d.fromRadians(0);

    public TurretTemplate(
        boolean isEnabled,
        ProfiledPIDController controller,
        SimpleMotorFeedforward feedforward,
        SubsystemConstants constants,
        EncoderConstants encoderConstants,
        MotorConstants... motorConstants
    ) {
        this.constants=constants;
        this.mainNum=constants.mainNum;
        this.controller=controller;
        this.feedforward=feedforward;
        this.minAngleRad=Math.toRadians(constants.lowerLimit);
        this.maxAngleRad=Math.toRadians(constants.upperLimit);
        this.conversionFactor=constants.conversionFactor;
        this.isEnabled=isEnabled;

        if (constants.encoderType == EncoderType.ABSOLUTE) {
            if (encoderConstants == null) {
                throw new NullPointerException("Encoder constants required for absolute encoder");
            }
            this.encoder = Optional.of(CANcoderEx.createWithConstants(encoderConstants));
        } else {
            this.encoder = Optional.empty();
        }
        
        
        this.motors = new TalonEx[motorConstants.length];
        
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
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/Reset Encoder", resetEncoderCommand());
    }

    @Override public void practiceWriters() {}
    @Override public void alerts() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Goal Angle (deg)", () -> getGoalAngle().getDegrees(), null);
        builder.addDoubleProperty("Current Angle (deg)", () -> getCurrentAngle().getDegrees(), null);
        builder.addDoubleProperty("Position Setpoint (deg)", this::getPositionSetpoint, null);
        builder.addDoubleProperty("Velocity Setpoint (deg/s)", this::getVelocitySetpoint, null);
        builder.addDoubleProperty("Current Velocity (rot/s)", ()-> getVelocity().in(RotationsPerSecond), null);
        builder.addDoubleProperty("Applied Voltage", this::getVoltage, null);
        builder.addDoubleProperty("Position Error (deg)", this::getPositionError, null);
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

    public synchronized void setGoalAngle(Rotation2d angle) {
        double angleRad = angle.getRadians();
        
        // Check if within valid range - if so, use as-is
        if (angleRad >= minAngleRad && angleRad <= maxAngleRad) {
            goalAngle = new Rotation2d(angleRad);
            controller.setGoal(goalAngle.getRadians());
            return;
        }
        
        // Out of range - try flipping 180°
        double flippedAngle = angleRad + Math.PI;
        
        // Normalize flipped angle to [-π, π]
        flippedAngle = MathUtil.angleModulus(flippedAngle);
        
        // Check if flipped angle is within range
        if (flippedAngle >= minAngleRad && flippedAngle <= maxAngleRad) {
            goalAngle = new Rotation2d(flippedAngle);
            controller.setGoal(goalAngle.getRadians());
            return;
        }
        
        // Neither original nor flipped works - clamp to nearest limit
        // This is a fallback that shouldn't normally happen
        double clamped = MathUtil.clamp(angleRad, minAngleRad, maxAngleRad);
        goalAngle = new Rotation2d(clamped);
        controller.setGoal(goalAngle.getRadians());
    }

    public synchronized Rotation2d getGoalAngle() {
        return goalAngle;
    }

    public double getVelocitySetpoint() {
        return Math.toDegrees(controller.getSetpoint().velocity);
    }

    public double getPositionSetpoint() {
        return Math.toDegrees(controller.getSetpoint().position);
    }

    public double getPositionError() {
        return Math.toDegrees(controller.getPositionError());
    }

    
    /* ---------------- Sensor Access ---------------- */

    public Rotation2d getCurrentAngle() {
        var rot = encoder
            .map(enc -> enc.getAbsolutePosition())
            .orElse(motors[mainNum].getPosition());

        return Rotation2d.fromRotations(rot.in(Rotations) * conversionFactor);
    }

    public AngularVelocity getVelocity() {
        return encoder
            .map(enc -> enc.getVelocity().times(conversionFactor))
            .orElse(motors[mainNum].getVelocity().times(conversionFactor)); 
    }
    
    public double getVoltage() {
        return motors[mainNum].getVoltage();
    }

    /* ---------------- Motor Control ---------------- */

    public void setVoltage(double voltage) {
        if (isEnabled) {
            for (TalonEx motor: motors) {
                motor.setVoltage(voltage);
            }
        }
    }

    public void setVoltage(Voltage voltage) {
        if (isEnabled) {
            for (TalonEx motor: motors) {
                motor.setVoltage(voltage);
            }
        }
    }

    public void resetEncoder() {
        if (hasExternalEncoder()) {
            return;
        } else {
            for (TalonEx motor: motors) {
                motor.resetEncoder(0);
            }
            setGoalAngle(Rotation2d.kZero);
        }
    }

    public Command resetEncoderCommand() {
        return new RunCommand(this::resetEncoder) {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };
    }

    /* ---------------- SysId ---------------- */

    private SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(new SysIdRoutine.Config(), 
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                    // Only apply voltage if within safe bounds
                    double currentAngle = getCurrentAngle().getRadians();
                    if (currentAngle >= minAngleRad && currentAngle <= maxAngleRad) {
                        setVoltage(voltage);
                    } else {
                        setVoltage(0); // Stop if at limits
                    }
                }, 
                (log) -> {
                    log.motor("motor")
                        .voltage(Volts.of(getVoltage()))
                        .angularPosition(this.getCurrentAngle().getMeasure())
                        .angularVelocity(this.getVelocity());
                }, 
                this
            )
        );
    }

    public Command getSysIdCommand() {
        return new SequentialCommandGroup(
            getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kForward)
                .until(this::isAtUpperLimit),
            new WaitCommand(0.1),
            getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kReverse)
                .until(this::isAtLowerLimit),
            new WaitCommand(0.1),
            getSysIdRoutine().dynamic(SysIdRoutine.Direction.kForward)
                .until(this::isAtUpperLimit),
            new WaitCommand(0.1),
            getSysIdRoutine().dynamic(SysIdRoutine.Direction.kReverse)
                .until(this::isAtLowerLimit)
        );
    }
    
    /* ---------------- Utility ---------------- */
    
    public TalonEx getMotor(){
        return motors[mainNum];
    }
    
    public TalonEx[] getAllMotor() {
        return motors;
    }

    public boolean atGoal(){
        return controller.atGoal();
    }

    private boolean isAtUpperLimit() {
        return getCurrentAngle().getRadians() >= maxAngleRad - 0.05; // 0.05 rad buffer
    }

    private boolean isAtLowerLimit() {
        return getCurrentAngle().getRadians() <= minAngleRad + 0.05; // 0.05 rad buffer
    }

    private boolean hasExternalEncoder() {
        return constants.encoderType == EncoderType.ABSOLUTE;
    }
}
