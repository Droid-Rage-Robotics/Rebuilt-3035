package frc.utility.template;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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

public class ElevatorTemplate extends SubsystemBase implements Dashboard, TelemetryUpdater {
    private final TalonEx[] motors;
    private final Optional<CANcoderEx> encoder;
    private final ProfiledPIDController controller;
    private final ElevatorFeedforward feedforward;

    // private final double maxPosition;
    // private final double minPosition;
    private final double conversionFactor;
    private final int mainNum;
    private final SubsystemConstants constants;

    private final boolean isEnabled;
    private final String name;
    
    public ElevatorTemplate(
        boolean isEnabled,
        ProfiledPIDController controller,
        ElevatorFeedforward feedforward,
        SubsystemConstants constants,
        EncoderConstants encoderConstants,
        MotorConstants... motorConstants
    ) {
        this.constants=constants;
        this.mainNum=constants.mainNum;
        this.controller=controller;
        this.feedforward=feedforward;
        this.name=constants.name;
        // this.maxPosition=constants.maxDistance.in(Meters);
        // this.minPosition=constants.minDistance.in(Meters);
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
        SmartDashboard.putData(constants.name + "/Reset Encoder", resetEncoderCommand());
    }

    @Override
    public void updateTelemetry() {
        Logger.recordOutput(name + "/Goal Position", getGoalPosition());
        Logger.recordOutput(name + "/Current Position", getPosition());
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
        double meter = getPosition().in(Meters);
        double pidOut = controller.calculate(meter);
        double ffOut = feedforward.calculate(controller.getSetpoint().velocity);

        setVoltage(pidOut + ffOut);
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    /* ---------------- Commands ---------------- */

    public Command setTargetPositionCommand(Distance value) {
        return new InstantCommand(() -> setTargetPosition(value));
    }

    /* ---------------- Manual Goal Control ---------------- */


    public void setTargetPosition(Distance position) {
        double clamped = MathUtil.clamp(
            position.in(Meters),
            constants.minDistance.in(Meters),
            constants.maxDistance.in(Meters)
        );
        controller.setGoal(clamped);
    }

    public Distance getGoalPosition() {
        return Meters.of(controller.getGoal().position);
    }

    public LinearVelocity getVelocitySetpoint() {
        return MetersPerSecond.of(controller.getSetpoint().velocity);
    }

    public Distance getPositionSetpoint() {
        return Meters.of(controller.getSetpoint().position);
    }

    public Distance getPositionError() {
        return Meters.of(controller.getPositionError());
    }
    
    /* ---------------- Sensor Access ---------------- */

    public Distance getPosition() {
        return Meters.of(encoder
            .map(enc -> enc.getAbsolutePosition().in(Radians) * conversionFactor)
            .orElse(motors[mainNum].getPosition().in(Radians) * conversionFactor)
        );
    }

    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(encoder
            .map(enc -> enc.getVelocity().in(RadiansPerSecond) * conversionFactor)
            .orElse(motors[mainNum].getVelocity().in(RadiansPerSecond) * conversionFactor));
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
            setTargetPosition(Meters.zero());
        }
    }

    public Command resetEncoderCommand() {
        return new InstantCommand(this::resetEncoder) {
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
                    // double currentPos = getPosition().in(Meters);
                    // if (currentPos >= minPosition || currentPos <= maxPosition) {
                        setVoltage(voltage);
                    // } else {
                    //     setVoltage(0); // Stop if at limits
                    // }
                }, 
                (log) -> {
                    log.motor("motor")
                        .voltage(Volts.of(getVoltage()))
                        .linearPosition(this.getPosition())
                        .linearVelocity(this.getVelocity());
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

    public boolean atGoal(){
        return controller.atGoal();
    }

    // private boolean isAtUpperLimit() {
    //     return getPosition().in(Meters) >= maxPosition - 0.05; // 0.05 cm buffer
    // }

    // private boolean isAtLowerLimit() {
    //     return getPosition().in(Meters) <= minPosition + 0.05; // 0.05 cm buffer
    // }

    private boolean hasExternalEncoder() {
        return constants.encoderType == EncoderType.ABSOLUTE;
    }
}