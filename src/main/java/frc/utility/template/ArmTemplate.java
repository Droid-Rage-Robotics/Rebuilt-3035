package frc.utility.template;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class ArmTemplate extends SubsystemBase implements Dashboard, TelemetryUpdater {
    protected final TalonEx[] motors;
    private final Optional<CANcoderEx> encoder;

    private final double minAngleRad;
    private final double maxAngleRad;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
        
    private final AtomicReference<Rotation2d> goalAngle = new AtomicReference<Rotation2d>(Rotation2d.kZero);

    
    protected final int mainNum;
    protected final String name;
    private final boolean isEnabled;
    private final SubsystemConstants constants;

    public ArmTemplate(
        boolean isEnabled,
        SubsystemConstants constants,
        EncoderConstants encoderConstants,
        MotorConstants... motorConstants
    ){
        this.isEnabled=isEnabled;
        this.constants=constants;
        this.minAngleRad=constants.minAngle.in(Radians);
        this.maxAngleRad=constants.maxAngle.in(Radians);
        this.mainNum=constants.mainNum;
        this.name=constants.name;

        this.motors = new TalonEx[motorConstants.length];
        
        for (MotorConstants m_motorConstants : motorConstants) {
            m_motorConstants.subsystem=this;
            m_motorConstants.isEnabled=isEnabled;
        }

        for (int i = 0; i < motorConstants.length; i++) {
            this.motors[i] = TalonEx.createWithConstants(motorConstants[i]);
        }

        for (int i = 0; i < motors.length; i++) {
            if (i != mainNum) {
                motors[i].getMotor().setControl(
                    new Follower(motors[mainNum].getMotor().getDeviceID(), MotorAlignmentValue.Aligned)
                    // 'false' = same direction, 'true' = opposite direction
                );
            }
        }

        var motorConfig = motors[mainNum].getConfig();

        if (constants.encoderType == EncoderType.ABSOLUTE || constants.encoderType == EncoderType.EXTERNAL) {
            if (encoderConstants == null) {
                throw new NullPointerException("Encoder constants required for external encoder");
            }

            this.encoder = Optional.of(CANcoderEx.createWithConstants(encoderConstants));

            motorConfig.Feedback.FeedbackRemoteSensorID=encoderConstants.deviceId;
            motorConfig.Feedback.FeedbackSensorSource=FeedbackSensorSourceValue.RemoteCANcoder;


        } else {
            this.encoder = Optional.empty();

            // motorConfig.Feedback.FeedbackRotorOffset=constants.offset;
        }

        motorConfig.Feedback.SensorToMechanismRatio=constants.gearRatio;
        
        // PID slot 0
        motorConfig.Slot0.kP = constants.kP;
        motorConfig.Slot0.kI = constants.kI;
        motorConfig.Slot0.kD = constants.kD;
        motorConfig.Slot0.kV = constants.kV;
        motorConfig.Slot0.kA = constants.kA;
        motorConfig.Slot0.kS = constants.kS;
        motorConfig.Slot0.kG = constants.kG;
        motorConfig.Slot0.GravityType=GravityTypeValue.Arm_Cosine;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = constants.maxVelocity.in(RotationsPerSecond);
        motorConfig.MotionMagic.MotionMagicAcceleration = constants.maxAcceleration.in(RotationsPerSecondPerSecond);
        motorConfig.MotionMagic.MotionMagicJerk = 0; // optional, set nonzero for S-curve smoothing

        motors[mainNum].getMotor().getConfigurator().apply(motorConfig);

        TelemetryUtils.registerDashboard(this);
        TelemetryUtils.registerTelemetry(this);
    }

    /* ---------------- Dashboard ---------------- */

    @Override
    public void elasticInit() {
        SmartDashboard.putData(name + "/Reset Encoder", resetEncoderCommand(0));
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    @Override
    public void updateTelemetry() {
        Logger.recordOutput(name + "/Goal Angle", getGoalAngle());
        Logger.recordOutput(name + "/Current Angle", getCurrentAngle());
        Logger.recordOutput(name + "/Position Setpoint", getPositionSetpoint());
        Logger.recordOutput(name + "/Velocity Setpoint", getVelocitySetpoint());
        Logger.recordOutput(name + "/Current Velocity", getVelocity());
        Logger.recordOutput(name + "/Applied Voltage", getVoltage());
        Logger.recordOutput(name + "/Position Error", getPositionError());
    }
    
    /* ---------------- Periodic Control Loop ---------------- */

    @Override
    public void periodic() {
        motors[mainNum].setControl(motionMagicRequest.withPosition(goalAngle.get().getMeasure()));
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
        double clamped = MathUtil.clamp(
            angle.getRadians(),
            minAngleRad,
            maxAngleRad
        );

        goalAngle.set(Rotation2d.fromRadians(clamped));
    }

    public Rotation2d getGoalAngle() {
        return goalAngle.get();
    }

    public Angle getPositionSetpoint() {
        return Rotations.of(motors[mainNum].getMotor().getClosedLoopReference().getValueAsDouble());
    }

    public AngularVelocity getVelocitySetpoint() {
        return RotationsPerSecond.of(motors[mainNum].getMotor().getClosedLoopReferenceSlope().getValueAsDouble());
    }

    public Angle getPositionError() {
        return Rotations.of(motors[mainNum].getMotor().getClosedLoopError().getValueAsDouble());
    }
    
    /* ---------------- Sensor Access ---------------- */

    public Rotation2d getCurrentAngle() {
        return new Rotation2d(motors[mainNum].getPosition());
    }

    public AngularVelocity getVelocity() {
        return motors[mainNum].getVelocity();
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
        motors[mainNum].setVoltage(voltage);
    }

    public void setVoltage(Voltage voltage) {
        if (isEnabled) {
            motors[mainNum].setVoltage(voltage);
        }
    }
    
    public void resetEncoder(Angle resetAngle) {
        switch(constants.encoderType){
            case ABSOLUTE: 
                return;
            case EXTERNAL: 
                encoder.get().resetPosition(resetAngle);
                motors[mainNum].resetEncoder(resetAngle.in(Rotations));
                break;
            case INTEGRATED: 
                motors[mainNum].resetEncoder(resetAngle.in(Rotations));
        }
    }

    public Command resetEncoderCommand(double resetDegree) {
        return new InstantCommand(()->resetEncoder(Degrees.of(resetDegree))).ignoringDisable(true);
    }

    /* ---------------- SysId ---------------- */

    public SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(0.5), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                null
            ), 
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                    // Only apply voltage if within safe bounds
                    // double currentAngle = getCurrentAngle().getRadians();
                    // if (currentAngle >= minAngleRad && currentAngle <= maxAngleRad) {
                        setVoltage(voltage);
                    // } else {
                    //     setVoltage(0); // Stop if at limits
                    // }
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
            getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kForward),
                // .until(this::isAtUpperLimit),
            new WaitCommand(0.1),
            getSysIdRoutine().quasistatic(SysIdRoutine.Direction.kReverse),
                // .until(this::isAtLowerLimit),
            new WaitCommand(0.1),
            getSysIdRoutine().dynamic(SysIdRoutine.Direction.kForward),
                // .until(this::isAtUpperLimit),
            new WaitCommand(0.1),
            getSysIdRoutine().dynamic(SysIdRoutine.Direction.kReverse)
                // .until(this::isAtLowerLimit)
        );
    }
    
    /* ---------------- Utility ---------------- */
    
    public TalonEx getMotor(){
        return motors[mainNum];
    }
    
    public TalonEx[] getAllMotor() {
        return motors;
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

    private boolean hasExternalEncoder() {
        return constants.encoderType == EncoderType.ABSOLUTE;
    }
}
