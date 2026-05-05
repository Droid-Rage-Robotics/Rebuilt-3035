package frc.utility.template;

import static edu.wpi.first.units.Units.*;

import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
import frc.utility.io.ArmIO;
import frc.utility.io.ArmIOInputsAutoLogged;
import frc.utility.template.Constants.ArmConstants;


public class ArmTemplate extends SubsystemBase implements Dashboard, TelemetryUpdater {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private final double minAngleRad;
    private final double maxAngleRad;
    private final Angle resetAngle;
    // private final SubsystemConstants constants;

    private final boolean isEnabled;
    private final AtomicReference<Angle> goalAngle = new AtomicReference<>(Degrees.zero());

    public boolean controlLoopEnabled = true;

    private final String name;
    private final Alert motorDisconnectedAlert;
    private final Alert tempAlert;

    public ArmTemplate(
            boolean isEnabled,
            ArmConstants constants,
            ArmIO io
    ) {
        // this.constants = constants;
        this.name = constants.name;
        this.minAngleRad = constants.minAngle.in(Radians);
        this.maxAngleRad = constants.maxAngle.in(Radians);
        this.resetAngle = constants.resetAngle;
        this.isEnabled = isEnabled;
        this.io = io;

        motorDisconnectedAlert =
            new Alert(name + " motor disconnected", AlertType.kWarning);
        tempAlert =
            new Alert(name + " motor temperature high", AlertType.kWarning);

        TelemetryUtils.registerDashboard(this);
        TelemetryUtils.registerTelemetry(this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);

        motorDisconnectedAlert.set(!inputs.motorConnected);
        tempAlert.set(inputs.temp.gt(Celsius.of(70)));

        if (controlLoopEnabled) {
            io.setGoalAngle(goalAngle.get());
        }
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    @Override
    public void elasticInit() {
        SmartDashboard.putData(name + "/Reset Encoder", resetEncoderCommand());
    }

    @Override
    public void practiceWriters() {}

    @Override
    public void alerts() {}

    @Override
    public void updateTelemetry() {
        Logger.recordOutput(name + "/Goal Angle", getGoalAngle().in(Degrees));
        Logger.recordOutput(name + "/Current Angle", getCurrentAngle().in(Degrees));
        Logger.recordOutput(name + "/Position Setpoint", getPositionSetpoint().in(Degrees));
        Logger.recordOutput(name + "/Velocity Setpoint", getVelocitySetpoint().in(RotationsPerSecond));
        Logger.recordOutput(name + "/Current Velocity", getVelocity().in(RotationsPerSecond));
        Logger.recordOutput(name + "/Applied Voltage", getVoltage());
        Logger.recordOutput(name + "/Position Error", getPositionError().in(Degrees));
    }

    public Command setTargetPositionCommand(Angle goalAngle) {
        return new InstantCommand(() -> setGoalAngle(goalAngle), this);
    }

    public void setGoalAngle(Angle angle) {
        double clamped = MathUtil.clamp(
            angle.in(Radians),
            minAngleRad,
            maxAngleRad
        );

        goalAngle.set(Radians.of(clamped));
    }

    public ArmIOInputsAutoLogged getInputs() {
        io.updateInputs(inputs);
        return inputs;
    }

    public Angle getGoalAngle() {
        return goalAngle.get();
    }

    public Angle getPositionSetpoint() {
        return inputs.closedLoopReference;
    }

    public AngularVelocity getVelocitySetpoint() {
        return inputs.closedLoopReferenceVelocity;
    }

    public Angle getPositionError() {
        return inputs.closedLoopError;
    }

    public Angle getCurrentAngle() {
        return inputs.position;
    }

    public AngularVelocity getVelocity() {
        return inputs.velocity;
    }

    public Voltage getVoltage() {
        return inputs.appliedVoltage;
    }

    public void setVoltage(double voltage) {
        if (isEnabled) {
            io.setVoltage(voltage);
        }
    }

    public void setVoltage(Voltage voltage) {
        if (isEnabled) {
            io.setVoltage(voltage);
        }
    }

    public void resetEncoder() {
        io.resetEncoder(resetAngle);
    }

    public void resetEncoder(Angle resetAngle) {
        io.resetEncoder(resetAngle);
    }

    public Command resetEncoderCommand() {
        return new InstantCommand(this::resetEncoder).ignoringDisable(true);
    }

    public Command resetEncoderCommand(Angle resetAngle) {
        return new InstantCommand(() -> resetEncoder(resetAngle)).ignoringDisable(true);
    }

    public SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(0.5),
                null,
                null
            ),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> log.motor("motor")
                    .voltage(inputs.appliedVoltage)
                    .angularPosition(inputs.position)
                    .angularVelocity(inputs.velocity),
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

    public boolean atGoal() {
        return Math.abs(getPositionError().in(Degrees)) < 5.0;
    }

    private boolean isAtUpperLimit() {
        return getCurrentAngle().in(Radians) >= maxAngleRad - 0.05;
    }

    private boolean isAtLowerLimit() {
        return getCurrentAngle().in(Radians) <= minAngleRad + 0.05;
    }

    public Command disableControlLoop() {
        return new InstantCommand(() -> controlLoopEnabled = false, this);
    }

    public Command enableControlLoop() {
        return new InstantCommand(() -> controlLoopEnabled = true, this);
    }
}