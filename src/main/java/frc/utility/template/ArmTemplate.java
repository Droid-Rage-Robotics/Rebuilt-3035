package frc.utility.template;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
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
    protected final ProfiledPIDController controller;
    protected final ArmFeedforward feedforward;
    private final double minAngleRad;
    private final double maxAngleRad;
    private final double conversionFactor;
    
    protected final int mainNum;
    protected final String name;
    private final Optional<CANcoderEx> encoder;
    private final boolean isEnabled;
    private final SubsystemConstants constants;

    public ArmTemplate(
        boolean isEnabled,
        ProfiledPIDController controller,
        ArmFeedforward feedforward,
        SubsystemConstants constants,
        EncoderConstants encoderConstants,
        MotorConstants... motorConstants
    ){
        this.isEnabled=isEnabled;
        this.constants=constants;
        this.controller=controller;
        this.feedforward=feedforward;
        this.minAngleRad=constants.minAngle.in(Radians);
        this.maxAngleRad=constants.maxAngle.in(Radians);
        this.conversionFactor=constants.conversionFactor;
        this.mainNum=constants.mainNum;
        this.name=constants.name;

        if (constants.encoderType == EncoderType.ABSOLUTE || constants.encoderType == EncoderType.EXTERNAL) {
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
        Logger.recordOutput(name + "/Position Error", controller.getPositionError());
    }
    
    /* ---------------- Periodic Control Loop ---------------- */

    @Override
    public void periodic() {
        double currentAngleRad = getCurrentAngle().getRadians();//+ Math.toRadians(constants.offset);
        
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

        controller.setGoal(clamped);
    }

    public Rotation2d getGoalAngle() {
        return Rotation2d.fromRadians(controller.getGoal().position);
    }

    public double getVelocitySetpoint() {
        return Math.toDegrees(controller.getSetpoint().velocity);
    }

    public double getPositionSetpoint() {
        return Math.toDegrees(controller.getSetpoint().position);
    }
    
    /* ---------------- Sensor Access ---------------- */

    public Rotation2d getCurrentAngle() {
        var rot = encoder
            .map(enc -> enc.getAbsolutePosition())
            .orElse(motors[mainNum].getPosition());

        return Rotation2d.fromRotations((rot.in(Rotations) * conversionFactor) + constants.offset);
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
    
    protected void setVoltage(double voltage) {
        double angle = getCurrentAngle().getRadians();
        if ((angle > maxAngleRad && voltage > 0) ||
            (angle < minAngleRad && voltage < 0)) {
            voltage = 0;
        }
        for (TalonEx motor: motors) {
            motor.setVoltage(voltage);
        }
    }

    public void setVoltage(Voltage voltage) {
        if (isEnabled) {
            for (TalonEx motor: motors) {
                motor.setVoltage(voltage);
            }
        }
    }
    
    public void resetEncoder(Angle resetAngle) {
        if (hasExternalEncoder()) {
            return;
        } else {
            for (TalonEx motor: motors) {
                motor.resetEncoder(resetAngle.in(Rotations));
                // motor.resetEncoder(resetAngle.in(Rotations)+Math.to);

            }
            setGoalAngle(Rotation2d.fromRotations(resetAngle.in(Rotation)+constants.offset));//new Rotation2d(resetAngle)
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

    public boolean atSetpoint(){
        return controller.atSetpoint();
    }
    
    public double getSetpointError() {
        return controller.getPositionError();
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
