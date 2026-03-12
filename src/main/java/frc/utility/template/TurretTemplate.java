package frc.utility.template;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utility.TelemetryUtils;
import frc.utility.TelemetryUtils.Dashboard;
import frc.utility.TelemetryUtils.TelemetryUpdater;
import frc.utility.devices.encoder.CANcoderEx;
import frc.utility.devices.encoder.EncoderConstants;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.TalonEx;
import frc.utility.template.SubsystemConstants.EncoderType;

public class TurretTemplate extends SubsystemBase implements Dashboard, TelemetryUpdater {
    private final TalonEx motor;
    private final Optional<CANcoderEx> encoder;

    private final double minAngleRad;
    private final double maxAngleRad;
    private final SubsystemConstants constants;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    private final boolean isEnabled;

    private final AtomicReference<Rotation2d> goalAngle = new AtomicReference<Rotation2d>(Rotation2d.kZero);

    private final MechanismLigament2d ligament;
    private final Mechanism2d mechanism;
    private final MechanismRoot2d center;

    private final String name;

    public TurretTemplate(
        boolean isEnabled,
        SubsystemConstants constants,
        EncoderConstants encoderConstants,
        MotorConstants motorConstants
    ) {
        this.constants=constants;
        this.name=constants.name;
        this.minAngleRad=constants.minAngle.in(Radians);
        this.maxAngleRad=constants.maxAngle.in(Radians);
        this.isEnabled=isEnabled;

        motorConstants.subsystem=this;
        motorConstants.isEnabled=isEnabled;
        
        this.motor = TalonEx.createWithConstants(motorConstants);
        this.mechanism = new Mechanism2d(constants.width, 10);

        center = mechanism.getRoot("center", 5, 5);

        ligament = new MechanismLigament2d(
            constants.name + "Ligma", 
            constants.length/2, 
            0, 
            1, 
            new Color8Bit(Color.kRed));

        center.append(ligament);

        var motorConfig = motor.getConfig();

        if (constants.encoderType == EncoderType.ABSOLUTE || constants.encoderType == EncoderType.EXTERNAL) {
            if (encoderConstants == null) {
                throw new NullPointerException("Encoder constants required for external encoder");
            }
            this.encoder = Optional.of(CANcoderEx.createWithConstants(encoderConstants));
            
            motorConfig.Feedback.FeedbackRemoteSensorID=encoderConstants.deviceId;
            motorConfig.Feedback.FeedbackSensorSource=FeedbackSensorSourceValue.RemoteCANcoder;
        } else {
            this.encoder = Optional.empty();
        }
        
        motorConfig.Feedback.SensorToMechanismRatio=constants.gearRatio;

        // PID slot 0
        motorConfig.Slot0.kP = constants.kP;
        motorConfig.Slot0.kI = constants.kI;
        motorConfig.Slot0.kD = constants.kD;
        motorConfig.Slot0.kV = constants.kV;
        motorConfig.Slot0.kA = constants.kA;
        motorConfig.Slot0.kS = constants.kS;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = constants.maxVelocity.in(RotationsPerSecond);
        motorConfig.MotionMagic.MotionMagicAcceleration = constants.maxAcceleration.in(RotationsPerSecondPerSecond);
        motorConfig.MotionMagic.MotionMagicJerk = 0; // optional, set nonzero for S-curve smoothing

        motor.getMotor().getConfigurator().apply(motorConfig);
        
        TelemetryUtils.registerDashboard(this);
        TelemetryUtils.registerTelemetry(this);
    }

    /* ---------------- Dashboard ---------------- */

    @Override
    public void elasticInit() {
        // SmartDashboard.putData(getName() + "/Mechanism", mechanism);
        SmartDashboard.putData(getName() + "/Reset Encoder", resetEncoderCommand());
    }

    @Override
    public void updateTelemetry() {
        Logger.recordOutput(name + "/Goal Angle", getGoalAngle());
        Logger.recordOutput(name + "/Current Angle", getCurrentAngle().getDegrees());
        Logger.recordOutput(name + "/Position Setpoint", getPositionSetpoint());
        Logger.recordOutput(name + "/Velocity Setpoint", getVelocitySetpoint());
        Logger.recordOutput(name + "/Current Velocity", getVelocity());
        Logger.recordOutput(name + "/Applied Voltage", getVoltage());
        Logger.recordOutput(name + "/Position Error", getPositionError());
    }

    @Override public void practiceWriters() {}
    @Override public void alerts() {}

    /* ---------------- Periodic Control Loop ---------------- */

    @Override
    public void periodic() {
        ligament.setAngle(getCurrentAngle());

        motor.setControl(motionMagicRequest.withPosition(goalAngle.get().getMeasure())); // isEnabled safety in motor file and auto unit conversion
    }

    @Override
    public void simulationPeriodic() {
    
    }

    /* ---------------- Commands ---------------- */

    public Command setTargetPositionCommand(Rotation2d goalAngle) {
        return runOnce(() -> setGoalAngle(goalAngle));
    }

    /* ---------------- Manual Goal Control ---------------- */

    public void setTargetPositionDegrees(double degrees) {
        setGoalAngle(Rotation2d.fromDegrees(degrees));
    }

    public void setGoalAngle(Rotation2d angle) {
        double angleRad = angle.getRadians();
        
        // Check if within valid range - if so, use as-is
        if (angleRad >= minAngleRad && angleRad <= maxAngleRad) {
            goalAngle.set(new Rotation2d(angleRad));
            return;
        }
        
        // Out of range - try flipping 180°
        double flippedAngle = angleRad + Math.PI;
        
        // Normalize flipped angle to [-π, π]
        flippedAngle = MathUtil.angleModulus(flippedAngle);
        
        // Check if flipped angle is within range
        if (flippedAngle >= minAngleRad && flippedAngle <= maxAngleRad) {
            goalAngle.set(new Rotation2d(flippedAngle));
            return;
        }
        
        // Neither original nor flipped works - clamp to nearest limit
        // This is a fallback that shouldn't normally happen
        double clamped = MathUtil.clamp(angleRad, minAngleRad, maxAngleRad);
        goalAngle.set(new Rotation2d(clamped));
    }

    public Rotation2d getGoalAngle() {
        return goalAngle.get();
    }

    public Angle getPositionSetpoint() {
        return Rotations.of(motor.getMotor().getClosedLoopReference().getValueAsDouble());
    }

    public AngularVelocity getVelocitySetpoint() {
        return RotationsPerSecond.of(motor.getMotor().getClosedLoopReferenceSlope().getValueAsDouble());
    }

    public Angle getPositionError() {
        return Rotations.of(motor.getMotor().getClosedLoopError().getValueAsDouble());
    }

    
    /* ---------------- Sensor Access ---------------- */

    public Rotation2d getCurrentAngle() {
        return new Rotation2d(motor.getPosition());
    }

    public AngularVelocity getVelocity() {
        return motor.getVelocity();
    }
    
    public double getVoltage() {
        return motor.getVoltage();
    }

    /* ---------------- Motor Control ---------------- */

    public void setVoltage(double voltage) {
        if (isEnabled) {
            motor.setVoltage(voltage);
        }
    }

    public void setVoltage(Voltage voltage) {
        if (isEnabled) {
            motor.setVoltage(voltage);
        }
    }

    public void resetEncoder() {
        switch(constants.encoderType){
            case ABSOLUTE: 
                return;
            case EXTERNAL: 
                encoder.get().resetPosition(Degrees.of(0));
                motor.resetEncoder(0);
                break;
            case INTEGRATED: 
                motor.resetEncoder(0);
        }
    }

    public Command resetEncoderCommand() {
        return new InstantCommand(this::resetEncoder).ignoringDisable(true);
    }

    /* ---------------- SysId ---------------- */

    private SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Second),
                Volts.of(1),
                Seconds.of(3),
                null
            ), 
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
        return motor;
    }

    // public boolean atGoal(){
    //     return controller.atGoal();
    // }

    private boolean isAtUpperLimit() {
        return getCurrentAngle().getRadians() >= maxAngleRad - 0.05; // 0.05 rad buffer
    }

    private boolean isAtLowerLimit() {
        return getCurrentAngle().getRadians() <= minAngleRad + 0.05; // 0.05 rad buffer
    }
}
