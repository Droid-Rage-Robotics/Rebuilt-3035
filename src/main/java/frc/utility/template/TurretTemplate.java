package frc.utility.template;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    // private final MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0);

    private final boolean isEnabled;

    private final AtomicReference<Angle> goalAngle = new AtomicReference<Angle>(Degrees.zero());

    private final String name;

    private boolean sysIdActive = false;

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

        // motorConfig.MotionMagic.MotionMagicExpo_kV = constants.kV;
        // motorConfig.MotionMagic.MotionMagicExpo_kA = constants.kA;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = constants.maxVelocity.in(RotationsPerSecond);
        motorConfig.MotionMagic.MotionMagicAcceleration = constants.maxAcceleration.in(RotationsPerSecondPerSecond);
        motorConfig.MotionMagic.MotionMagicJerk = constants.maxJerk; // optional, set nonzero for S-curve smoothing

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
        Logger.recordOutput(name + "/Goal Angle", getGoalAngle().in(Degrees));
        Logger.recordOutput(name + "/Current Angle", getCurrentAngle().in(Degrees));
        // Logger.recordOutput(name + "/Position Setpoint", getPositionSetpoint());
        // Logger.recordOutput(name + "/Velocity Setpoint", getVelocitySetpoint());
        // Logger.recordOutput(name + "/Current Velocity", getVelocity());
        // Logger.recordOutput(name + "/Applied Voltage", getVoltage());
        Logger.recordOutput(name + "/Position Error", getPositionError().in(Degrees));
    }

    @Override public void practiceWriters() {}
    @Override public void alerts() {}

    /* ---------------- Periodic Control Loop ---------------- */

    @Override
    public void periodic() {
        if (!sysIdActive) {
            motor.setControl(motionMagicRequest.withPosition(goalAngle.get())); // isEnabled safety in motor file and auto unit conversion
        }
    }

    @Override
    public void simulationPeriodic() {
    
    }

    /* ---------------- Commands ---------------- */

    public Command setTargetPositionCommand(Angle goalAngle) {
        return new InstantCommand(() -> setGoalAngle(goalAngle));
    }

    /* ---------------- Manual Goal Control ---------------- */
    // public void setGoalAngle(Angle angle) {
    //     double clamped = MathUtil.clamp(
    //         angle.in(Radians),
    //         minAngleRad,
    //         maxAngleRad
    //     );

    //     goalAngle.set(Radians.of(clamped));
    // }
    public void setGoalAngle(Angle angle) {
        double angleRad = MathUtil.inputModulus(angle.in(Radians), 0, 2 * Math.PI);

        double clamped;
        if (angleRad > maxAngleRad) {
            // In the forbidden zone — figure out which endpoint is closer
            double distToMax = angleRad - maxAngleRad;
            double distToMin = (2 * Math.PI) - angleRad + minAngleRad;
            clamped = (distToMax <= distToMin) ? maxAngleRad : minAngleRad;
        } else {
            clamped = MathUtil.clamp(angleRad, minAngleRad, maxAngleRad);
        }

        goalAngle.set(Radians.of(clamped));
    }

    public Angle getGoalAngle() {
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

    public Angle getCurrentAngle() {
        return motor.getPosition();
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
                Volts.of(0.6).per(Second),
                Volts.of(3),
                // Seconds.of(3),
                null
            ), 
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                    // Only apply voltage if within safe bounds
                    double currentAngle = getCurrentAngle().in(Radians);
                    if (currentAngle >= minAngleRad && currentAngle <= maxAngleRad) {
                        setVoltage(voltage);
                    } else {
                        setVoltage(0); // Stop if at limits
                    }
                }, 
                (log) -> {
                    log.motor("motor")
                        .voltage(Volts.of(getVoltage()))
                        .angularPosition(this.getCurrentAngle())
                        .angularVelocity(this.getVelocity());
                }, 
                this
            )
        );
    }

    public Command getSysIdCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> sysIdActive = true),
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
                .until(this::isAtLowerLimit),
            new InstantCommand(() -> sysIdActive = false)            
        );
    }
    
    /* ---------------- Utility ---------------- */
    
    public TalonEx getMotor(){
        return motor;
    }

    public boolean atGoal(){
        return (Math.abs(getPositionError().in(Degrees)) < 5);
    }

    private boolean isAtUpperLimit() {
        return getCurrentAngle().in(Radians) >= maxAngleRad - Units.degreesToRadians(20); // 0.05 rad buffer
    }

    private boolean isAtLowerLimit() {
        return getCurrentAngle().in(Radians) <= minAngleRad + Units.degreesToRadians(20); // 0.05 rad buffer
    }
}
